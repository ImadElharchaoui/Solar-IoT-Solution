#!/usr/bin/env python3
"""
BLE-to-MQTT gateway for MPPT ESP32 nodes.

Each instance manages one ESP32 peripheral. The device serial number is
operator-configured via --serial (the RPi knows which controller it is
physically wired next to). If omitted, the gateway still works but MQTT
topics use "unknown" until a DeviceInfo message provides it - however
DeviceInfo no longer carries serial so always pass --serial.

Usage:
    python3 rpi_ble_gateway.py --serial 0503-18 --broker 192.168.1.10 --zone site1
"""
import argparse
import asyncio
import logging
import struct
import time
import sys
import os
import socket
from typing import Optional

from bleak import BleakClient, BleakScanner
import paho.mqtt.client as mqtt

_REPO_ROOT = os.path.normpath(os.path.join(os.path.dirname(__file__), ".."))
if _REPO_ROOT not in sys.path:
    sys.path.insert(0, _REPO_ROOT)

try:
    from proto import mppt_pb2
except ImportError:
    sys.exit(
        f"Cannot import proto.mppt_pb2.\n"
        f"Run `make proto` inside parser/ to generate it.\n"
        f"Expected: {os.path.join(_REPO_ROOT, 'proto', 'mppt_pb2.py')}"
    )

# ── BLE constants ──────────────────────────────────────────────────────────────
SVC_UUID = "6e400001-b5a3-f393-e0a9-e50e24dcca9e"
TX_UUID  = "6e400003-b5a3-f393-e0a9-e50e24dcca9e"  # ESP32 -> RPi (notify)
RX_UUID  = "6e400002-b5a3-f393-e0a9-e50e24dcca9e"  # RPi -> ESP32 (write)

MSG_TELEMETRY    = 0x01
MSG_FAULT_STATUS = 0x02
MSG_DEVICE_INFO  = 0x03
MSG_SETTINGS     = 0x04
MSG_DATALOG      = 0x05
MSG_ACK          = 0x11
MSG_CMD          = 0x10

CHUNK_FIRST = 0x00
CHUNK_CONT  = 0x01
CHUNK_LAST  = 0x80

BLE_DEVICE_NAME  = "MPPT-Gateway"
BLE_SCAN_TIMEOUT = 15.0      # seconds
BLE_TARGET_MTU   = 247       # 247 is widely supported (244 byte payload)
CMD_WRITE_DELAY  = 0.02      # seconds between BLE write chunks

log = logging.getLogger("mppt-ble")


# ── BLE framing ────────────────────────────────────────────────────────────────

class FrameReassembler:
    """
    Reassembles chunked BLE notifications into complete framed messages.

    Chunk wire format:
      byte 0: tag  0x00=FIRST, 0x01=CONT, 0x80=LAST
      byte 1+: data

    Frame format (assembled buffer):
      byte 0:    msg_type (uint8)
      bytes 1-2: payload_len (uint16 little-endian)
      bytes 3+:  payload
    """

    def __init__(self, timeout: float = 10.0):
        self._buf: bytearray = bytearray()
        self._busy: bool = False
        self._last_chunk_time: float = 0.0
        self._timeout = timeout

    def feed(self, data: bytes) -> Optional[tuple[int, bytes]]:
        if not data:
            return None

        now = time.monotonic()
        tag   = data[0]
        chunk = data[1:]

        # Watchdog: drop partial frame if peer went silent too long
        if self._busy and (now - self._last_chunk_time > self._timeout):
            log.warning("Reassembler timeout, dropping %d buffered bytes", len(self._buf))
            self._buf.clear()
            self._busy = False

        self._last_chunk_time = now

        if tag == CHUNK_FIRST:
            self._buf  = bytearray(chunk)
            self._busy = True
        elif tag == CHUNK_CONT:
            if not self._busy:
                log.debug("Stray CONT chunk, ignoring")
                return None
            self._buf.extend(chunk)
            return None  # never complete on CONT
        elif tag == CHUNK_LAST:
            if not self._busy:
                log.debug("Stray LAST chunk, ignoring")
                return None
            self._buf.extend(chunk)
            self._busy = False
        else:
            log.warning("Unknown chunk tag 0x%02X, ignoring", tag)
            return None

        # Check if the assembled buffer contains a complete frame.
        # This fires on CHUNK_FIRST (single-chunk) or CHUNK_LAST.
        if len(self._buf) < 3:
            return None

        payload_len = struct.unpack_from("<H", self._buf, 1)[0]
        if len(self._buf) < 3 + payload_len:
            return None  # truncated frame, wait

        msg_type = self._buf[0]
        payload  = bytes(self._buf[3: 3 + payload_len])
        self._buf.clear()
        return (msg_type, payload)


def build_framed_packet(msg_type: int, payload: bytes,
                        mtu: int = BLE_TARGET_MTU) -> list[bytes]:
    """Split a proto payload into BLE-sized chunks with framing tags."""
    frame          = bytes([msg_type]) + struct.pack("<H", len(payload)) + payload
    # GATT overhead is 3 bytes (Opcode + Handle). We also use 1 byte for our tag.
    # Total PDU size must be <= MTU.
    chunk_data_max = mtu - 4
    chunks         = []
    offset         = 0

    while offset < len(frame):
        take    = min(len(frame) - offset, chunk_data_max)
        is_last = offset + take >= len(frame)
        is_first = offset == 0

        if is_first and is_last:
            tag = CHUNK_FIRST           # single-chunk message
        elif is_first:
            tag = CHUNK_FIRST
        elif is_last:
            tag = CHUNK_LAST
        else:
            tag = CHUNK_CONT

        chunks.append(bytes([tag]) + frame[offset: offset + take])
        offset += take

    return chunks


# ── Gateway ────────────────────────────────────────────────────────────────────

class MpptBleGateway:

    def __init__(self, broker: str, port: int, zone: str,
                 gateway_id: str, serial: Optional[str]):
        self.broker     = broker
        self.port       = port
        self.zone       = zone
        self.gateway_id = gateway_id

        self._serial         = serial  # operator-configured; may be None
        self._known_address: Optional[str] = None
        self._client: Optional[BleakClient] = None
        self._reassembler    = FrameReassembler(timeout=10.0)
        self._cmd_queue: asyncio.Queue = asyncio.Queue()
        self._mqtt_connected = False
        self._loop           = asyncio.get_event_loop()
        self._mqtt           = self._setup_mqtt()

    # ── MQTT setup ────────────────────────────────────────────────────────────

    def _setup_mqtt(self) -> mqtt.Client:
        client = mqtt.Client(
            mqtt.CallbackAPIVersion.VERSION2,
            client_id=f"ble_gw_{self.gateway_id}"
        )
        client.on_connect    = lambda *a: self._loop.call_soon_threadsafe(self._on_mqtt_connect, *a)
        client.on_message    = lambda *a: self._loop.call_soon_threadsafe(self._on_mqtt_message, *a)
        client.on_disconnect = lambda *a: self._loop.call_soon_threadsafe(self._on_mqtt_disconnect, *a)
        client.reconnect_delay_set(min_delay=1, max_delay=30)
        client.connect_async(self.broker, self.port, keepalive=60)
        client.loop_start()
        return client

    def _on_mqtt_connect(self, client, userdata, flags, reason, props):
        self._mqtt_connected = True
        log.info("MQTT connected")
        client.subscribe("mppt/+/+/+/cmd", qos=1)
        if self._serial:
            self._publish_online(True)

    def _on_mqtt_disconnect(self, client, userdata, flags, reason, props):
        self._mqtt_connected = False
        log.warning("MQTT disconnected: %s", reason)

    def _on_mqtt_message(self, client, userdata, msg):
        parts = msg.topic.split("/")
        if len(parts) >= 5 and parts[4] == "cmd":
            # Accept commands for our serial or if serial unknown
            if not self._serial or parts[3] == self._serial:
                log.info("Queuing CMD for %s (%d B)", parts[3], len(msg.payload))
                self._cmd_queue.put_nowait(msg.payload)

    def _publish(self, subtopic: str, payload: bytes,
                 retain: bool = False, qos: int = 0):
        if not self._serial:
            log.debug("Serial unknown, dropping publish to %s", subtopic)
            return
        topic = f"mppt/{self.zone}/{self.gateway_id}/{self._serial}/{subtopic}"
        self._mqtt.publish(topic, payload, qos=qos, retain=retain)
        log.debug("MQTT -> %s (%d B)", topic, len(payload))

    def _publish_online(self, online: bool):
        if not self._serial:
            return
        topic = f"mppt/{self.zone}/{self.gateway_id}/{self._serial}/online"
        self._mqtt.publish(topic, "1" if online else "0", qos=1, retain=True)

    # ── BLE dispatch ──────────────────────────────────────────────────────────

    def _dispatch(self, msg_type: int, payload: bytes):
        try:
            if msg_type == MSG_TELEMETRY:
                log.info("Telemetry (%d B)", len(payload))
                self._publish("state", payload)

            elif msg_type == MSG_FAULT_STATUS:
                log.info("FaultStatus (%d B)", len(payload))
                self._publish("faults", payload, retain=True, qos=1)

            elif msg_type == MSG_DEVICE_INFO:
                m = mppt_pb2.DeviceInfo()
                m.ParseFromString(payload)
                log.info("DeviceInfo: hw_version=%d fw=%d type=%s",
                         m.hw_version, m.firmware_version, m.device_type)
                # Serial is operator-configured via --serial; DeviceInfo is
                # published as-is so Node-RED gets hw/fw/voltage metadata.
                self._publish("info", payload, retain=True, qos=1)

            elif msg_type == MSG_SETTINGS:
                log.info("Settings (%d B)", len(payload))
                self._publish("settings", payload, retain=True, qos=1)

            elif msg_type == MSG_DATALOG:
                log.info("Datalog (%d B)", len(payload))
                m = mppt_pb2.DataloggerPayload()
                m.ParseFromString(payload)
                self._split_and_publish_datalog(m)

            elif msg_type == MSG_ACK:
                log.info("ACK (%d B)", len(payload))
                self._publish("ack", payload, qos=1)

            else:
                log.warning("Unknown msg_type 0x%02X (%d B)", msg_type, len(payload))

        except Exception as e:
            log.error("Dispatch error for type 0x%02X: %s", msg_type, e)

    def _split_and_publish_datalog(self, m: "mppt_pb2.DataloggerPayload"):
        """Split DataloggerPayload into three retained sub-topics."""
        summary = mppt_pb2.DataloggerPayload()
        summary.CopyFrom(m)
        summary.ClearField("daily_logs")
        summary.ClearField("monthly_logs")
        self._publish("datalog/summary", summary.SerializeToString(),
                      retain=True, qos=1)

        if m.daily_logs:
            daily = mppt_pb2.DataloggerPayload()
            daily.timestamp = m.timestamp
            daily.daily_logs.extend(m.daily_logs)
            self._publish("datalog/daily", daily.SerializeToString(),
                          retain=True, qos=1)

        if m.monthly_logs:
            monthly = mppt_pb2.DataloggerPayload()
            monthly.timestamp = m.timestamp
            monthly.monthly_logs.extend(m.monthly_logs)
            self._publish("datalog/monthly", monthly.SerializeToString(),
                          retain=True, qos=1)

    def _on_notify(self, handle, data: bytearray):
        result = self._reassembler.feed(bytes(data))
        if result:
            msg_type, payload = result
            self._dispatch(msg_type, payload)

    # ── BLE connection ────────────────────────────────────────────────────────

    def _on_ble_disconnected(self, client: BleakClient):
        log.warning("BLE disconnected from %s", client.address)
        self._publish_online(False)

    async def run(self):
        if self.gateway_id == "auto":
            self.gateway_id = socket.gethostname()

        # Use cached address for fast reconnect after first discovery
        address = self._known_address
        if not address:
            log.info("Scanning for '%s'...", BLE_DEVICE_NAME)
            dev = await BleakScanner.find_device_by_name(
                BLE_DEVICE_NAME, timeout=BLE_SCAN_TIMEOUT
            )
            if not dev:
                raise RuntimeError(f"'{BLE_DEVICE_NAME}' not found in scan")
            address = dev.address
            log.info("Found %s at %s", dev.name, address)

        self._known_address = address
        log.info("Connecting to %s...", address)

        async with BleakClient(
            address,
            disconnected_callback=self._on_ble_disconnected
        ) as client:
            self._client = client

            # ── Negotiate higher MTU ──────────────────────────────────────────
            # Default is 23 bytes / 20 usable which is far too small.
            try:
                # 1. On Linux/BlueZ, accessing 'services' triggers discovery and MTU sync
                _ = client.services 
                
                # Give BlueZ a moment to propagate the MTU property to DBus
                await asyncio.sleep(1.0)

                # 2. Call _acquire_mtu() to sync negotiated value from DBus.
                # If this succeeds, Bleak sets its internal _mtu_size, 
                # which solves the UserWarning for subsequent accesses.
                if hasattr(client, "_acquire_mtu"):
                    await client._acquire_mtu()
                
                log.info("Connected to %s, MTU=%d (Payload=%d)", 
                         address, client.mtu_size, client.mtu_size - 3)
            except Exception as e:
                log.warning("MTU sync/discovery failed (%s), using MTU=%d", e, client.mtu_size)

            await client.start_notify(TX_UUID, self._on_notify)

            # Some BlueZ versions only update the property after first traffic
            if hasattr(client, "_acquire_mtu"):
                try:
                    await asyncio.sleep(0.5)
                    await client._acquire_mtu()
                    log.info("MTU after notification sync: %d", client.mtu_size)
                except Exception:
                    pass

            self._publish_online(True)
            actual_mtu = client.mtu_size

            while client.is_connected:
                try:
                    cmd_bytes = await asyncio.wait_for(
                        self._cmd_queue.get(), timeout=1.0
                    )
                    log.info("Forwarding CMD (%d B) to ESP32", len(cmd_bytes))
                    for chunk in build_framed_packet(MSG_CMD, cmd_bytes, mtu=actual_mtu):
                        await client.write_gatt_char(RX_UUID, chunk, response=False)
                        await asyncio.sleep(CMD_WRITE_DELAY)
                except asyncio.TimeoutError:
                    pass  # normal - just checking is_connected
                except Exception as e:
                    log.error("BLE write error: %s", e)
                    break

    def cleanup(self):
        log.info("Shutting down MQTT")
        self._mqtt.loop_stop()
        self._mqtt.disconnect()


# ── Entry point ────────────────────────────────────────────────────────────────

async def main_loop():
    parser = argparse.ArgumentParser(description="MPPT BLE-to-MQTT gateway")
    parser.add_argument("--broker",  default="localhost",
                        help="MQTT broker host")
    parser.add_argument("--port",    default=1883, type=int,
                        help="MQTT broker port")
    parser.add_argument("--zone",    default="default",
                        help="MQTT topic zone segment")
    parser.add_argument("--gateway", default="auto",
                        help="Gateway ID (default: hostname)")
    parser.add_argument("--serial",  default=None,
                        help="Device serial number e.g. 0503-18 (required for MQTT topics)")
    parser.add_argument("--esp32",   default=None,
                        help="ESP32 BLE address (skip scan if known)")
    args = parser.parse_args()

    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s %(levelname)-7s %(message)s"
    )

    if not args.serial:
        log.warning("No --serial provided. MQTT publishes will be dropped until "
                    "a DeviceInfo message is received (which no longer carries serial). "
                    "Pass --serial <MMSS-MO> to fix this.")

    gw = MpptBleGateway(
        broker=args.broker,
        port=args.port,
        zone=args.zone,
        gateway_id=args.gateway,
        serial=args.serial,
    )

    # Pre-seed address if provided to skip scan on first connect
    if args.esp32:
        gw._known_address = args.esp32

    log.info("Gateway %s | zone=%s | serial=%s | broker=%s:%d",
             gw.gateway_id, gw.zone, gw._serial or "unknown",
             gw.broker, gw.port)

    try:
        while True:
            try:
                await gw.run()
            except Exception as e:
                log.error("Gateway error: %s - retrying in 5s", e)
                # Clear cached address on repeated failures so we re-scan
                # (device may have changed address after power cycle)
                await asyncio.sleep(5)
    finally:
        gw.cleanup()


if __name__ == "__main__":
    try:
        asyncio.run(main_loop())
    except KeyboardInterrupt:
        pass
