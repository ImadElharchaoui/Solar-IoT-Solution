#!/usr/bin/env python3
import math
import os
import random
import sys
import threading
import time
from datetime import datetime, timezone

import paho.mqtt.client as mqtt
from paho.mqtt.enums import CallbackAPIVersion

_REPO_ROOT = os.path.normpath(os.path.join(os.path.dirname(__file__), "..", ".."))
if _REPO_ROOT not in sys.path:
    sys.path.insert(0, _REPO_ROOT)

try:
    from proto import mppt_pb2
except ImportError:
    sys.exit(
        f"Cannot import proto.mppt_pb2.\n"
        f"Run `make proto` (or `make`) inside parser/ to generate it, then retry.\n"
        f"Expected location: {os.path.join(_REPO_ROOT, 'proto', 'mppt_pb2.py')}"
    )

MQTT_HOST = "localhost"
MQTT_PORT = 1883
PUBLISH_INTERVAL_S = 5  # live telemetry cadence
NEW_DAY_DELAY_S = 60  # seconds before a fresh day-30 is re-injected

TOPIC_STATE = "mppt/default/{gw}/{serial}/state"
TOPIC_ONLINE = "mppt/default/{gw}/{serial}/online"
TOPIC_DATALOG = "mppt/default/{gw}/{serial}/datalog"
TOPIC_INFO = "mppt/default/{gw}/{serial}/info"
TOPIC_SETTINGS = "mppt/default/{gw}/{serial}/settings"
TOPIC_CMD = "mppt/+/+/{serial}/cmd"  # subscribe pattern per device
TOPIC_ACK = "mppt/default/{gw}/{serial}/ack"

TOTAL_DAYS = 30
TOTAL_MONTHS = 12

_BAT_TYPES = ["AGM", "Liquid", "LiFePO4 High", "LiFePO4 Med", "LiFePO4 Low"]
_NIGHT_MODES = ["Off", "D2D", "DD", "MN"]


def _rnd(lo: float, hi: float, dec: int = 2) -> float:
    return round(lo + random.random() * (hi - lo), dec)


def _rnd_int(lo: int, hi: int) -> int:
    return random.randint(lo, hi)


MODE_SCHEDULE = [
    ("MPP Tracking", 120),
    ("Boost", 60),
    ("Float", 180),
]


class SimDevice:
    def __init__(self, serial: str, hw: int, gateway: str, start_mode: str):
        self.serial = serial
        self.hw = hw
        self.gateway = gateway

        self._mode_idx = next(
            i for i, (m, _) in enumerate(MODE_SCHEDULE) if m == start_mode
        )
        self._mode_timer = 0.0
        self.charge_mode = start_mode

        self.soc = _rnd(40.0, 90.0, 1)
        self.vbat = _rnd(12.8, 13.8, 3)
        self.vpv = _rnd(15.0, 20.0, 3)
        self.vpv_target = _rnd(14.5, 16.5, 3)
        self.ichg = _rnd(1.0, 5.0, 2)
        self.iload = _rnd(0.5, 3.0, 2)
        self.temp_int = _rnd(22.0, 35.0, 1)
        self.temp_ext = _rnd(18.0, 32.0, 1)
        self.led_v = _rnd(11.5, 13.5, 3) if hw == 3 else 0.0
        self.led_i = _rnd(0.01, 0.15, 2) if hw == 3 else 0.0

        self.faults = {
            "battery_over_voltage": False,
            "pv_over_voltage": False,
            "controller_over_temp": False,
            "charge_over_current": False,
            "lvd_active": False,
            "over_discharge_current": False,
            "battery_over_temp": False,
            "battery_under_temp": False,
        }

        self._last_t = time.time()
        self._startup_time = time.time()
        self._new_day_sent = False
        self._settings_override: mppt_pb2.DeviceSettings | None = None

        self._burst_daily = [self._daily_row(d) for d in range(1, TOTAL_DAYS + 1)]
        self._burst_monthly = [self._monthly_row(m) for m in range(1, TOTAL_MONTHS + 1)]
        self._burst_sent = False

    @staticmethod
    def _jitter(v: float, pct: float = 0.02) -> float:
        return v * (1.0 + random.uniform(-pct, pct))

    @staticmethod
    def _sine(t: float, period: float, lo: float, hi: float) -> float:
        return lo + (hi - lo) * (0.5 + 0.5 * math.sin(2 * math.pi * t / period))

    def step(self):
        now = time.time()
        dt = now - self._last_t
        self._last_t = now

        self._mode_timer += dt
        _, dur = MODE_SCHEDULE[self._mode_idx]
        if self._mode_timer >= dur:
            self._mode_timer = 0.0
            self._mode_idx = (self._mode_idx + 1) % len(MODE_SCHEDULE)
        self.charge_mode, _ = MODE_SCHEDULE[self._mode_idx]

        if self.charge_mode == "MPP Tracking":
            vpv_base = self._sine(now, 300, 16.0, 20.0)
            ichg_base = self._sine(now, 240, 4.0, 10.0)
            self.vpv_target = self._jitter(15.5, 0.03)
        elif self.charge_mode == "Boost":
            vpv_base = self._sine(now, 300, 14.0, 17.0)
            ichg_base = self._sine(now, 240, 2.0, 6.0)
            self.vpv_target = self._jitter(14.4, 0.02)
        else:
            vpv_base = self._sine(now, 300, 9.0, 14.0)
            ichg_base = self._sine(now, 240, 0.1, 1.5)
            self.vpv_target = self._jitter(13.8, 0.02)

        self.vpv = self._jitter(vpv_base, 0.02)
        self.ichg = max(0.0, self._jitter(ichg_base, 0.05))
        net_ah = (self.ichg - self.iload) * dt / 3600.0
        self.soc = max(0.0, min(100.0, self.soc + net_ah / 1.28))
        self.vbat = 11.0 + (self.soc / 100.0) * 3.0 + self._jitter(0.0, 0.005)

        if self.charge_mode == "Boost":
            self.vbat = min(14.4, self.vbat + self._jitter(0.3, 0.05))
        elif self.charge_mode == "Float":
            floor = 13.6 if self.soc > 80 else self.vbat
            self.vbat = min(13.8, max(self.vbat, floor))

        self.iload = max(0.5, min(3.0, self.iload + random.uniform(-0.1, 0.1)))
        self.temp_int = self._jitter(self._sine(now, 600, 22.0, 42.0), 0.01)
        self.temp_ext = self._jitter(self._sine(now, 900, 18.0, 38.0), 0.01)

        if self.hw == 3:
            night = (int(now / 60) % 20) < 12
            self.led_v = self._jitter(12.0, 0.02) if night else self._jitter(0.2, 0.10)
            self.led_i = self._jitter(0.85, 0.05) if night else 0.0

        for fault in list(self.faults):
            self.faults[fault] = random.random() < 0.15
        if self.soc < 5.0:
            self.faults["lvd_active"] = True
        self.faults["controller_over_temp"] = self.temp_int > 45.0
        self.faults["battery_over_temp"] = self.temp_int > 50.0
        self.faults["battery_under_temp"] = self.temp_ext < 12.0

    def _daily_row(self, day: int) -> dict:
        base_vbat = _rnd(12.0, 14.2, 3)
        base_vpv = _rnd(8.0, 20.0, 3)
        return {
            "index": day,
            "vbat_min_v": round(base_vbat - _rnd(0.5, 1.5, 3), 3),
            "vbat_max_v": round(base_vbat + _rnd(0.2, 0.8, 3), 3),
            "vpv_min_v": round(max(0.0, base_vpv - _rnd(5.0, 10.0, 3)), 3),
            "vpv_max_v": base_vpv,
            "ah_charge": _rnd(10.0, 70.0, 1),
            "ah_load": _rnd(8.0, 60.0, 1),
            "il_max_a": _rnd(0.5, 12.0, 2),
            "ipv_max_a": _rnd(1.0, 10.0, 2),
            "soc_pct": float(_rnd_int(5, 100)),
            "ext_temp_max_c": _rnd_int(18, 45),
            "ext_temp_min_c": _rnd_int(5, 20),
            "nightlength_min": _rnd_int(540, 800),
            "flag_full_charge": day % 3 == 0,
            "flag_low_soc": random.random() > 0.8,
        }

    def _monthly_row(self, month: int) -> dict:
        season = 0.5 + 0.5 * math.sin(math.pi * (month - 1) / 11)
        return {
            "index": month,
            "vbat_min_v": _rnd(11.2, 12.5, 3),
            "vbat_max_v": _rnd(13.5, 14.2, 3),
            "vpv_min_v": _rnd(3.0, 7.0, 3),
            "vpv_max_v": _rnd(17.0, 22.0, 3),
            "ah_charge": round(200.0 + season * 700.0 + _rnd(-50, 50, 1), 1),
            "ah_load": round(150.0 + season * 500.0 + _rnd(-30, 30, 1), 1),
            "il_max_a": _rnd(1.0, 4.0, 2),
            "ipv_max_a": _rnd(2.0, 8.0, 2),
            "soc_pct": float(_rnd_int(20, 90)),
            "ext_temp_max_c": _rnd_int(20, 50),
            "ext_temp_min_c": _rnd_int(0, 20),
            "nightlength_min": _rnd_int(540, 720),
        }

    def apply_settings(self, ps: mppt_pb2.DeviceSettings, client: mqtt.Client):
        """Apply partial DeviceSettings safely using HasField() for optional fields."""
        print(f"  [{self.serial}] Received partial settings update:")

        if self._settings_override is None:
            self._settings_override = mppt_pb2.DeviceSettings()
            # Load current defaults
            tmp = mppt_pb2.DeviceSettings()
            tmp.ParseFromString(self.get_settings_proto())
            self._settings_override.CopyFrom(tmp)

        updated = []

        # === Safe partial update using HasField() for optional proto3 fields ===
        if ps.HasField("battery_type_index"):
            self._settings_override.battery_type_index = ps.battery_type_index
            bat = (
                _BAT_TYPES[ps.battery_type_index]
                if ps.battery_type_index < len(_BAT_TYPES)
                else str(ps.battery_type_index)
            )
            print(f"    battery type    -> {bat}")
            updated.append("battery")

        if ps.HasField("capacity_ah"):
            self._settings_override.capacity_ah = ps.capacity_ah
            print(f"    capacity        -> {ps.capacity_ah} Ah")
            updated.append("capacity")

        if ps.HasField("lvd_voltage_mv"):
            self._settings_override.lvd_voltage_mv = ps.lvd_voltage_mv
            print(f"    LVD voltage     -> {ps.lvd_voltage_mv / 1000:.2f} V")
            updated.append("lvd")

        if ps.HasField("lvd_mode_voltage"):
            self._settings_override.lvd_mode_voltage = ps.lvd_mode_voltage
            print(
                f"    LVD mode        -> {'voltage' if ps.lvd_mode_voltage else 'SOC'}"
            )
            updated.append("lvd_mode")

        if ps.HasField("night_mode_index"):
            self._settings_override.night_mode_index = ps.night_mode_index
            night = (
                _NIGHT_MODES[ps.night_mode_index]
                if ps.night_mode_index < len(_NIGHT_MODES)
                else str(ps.night_mode_index)
            )
            print(f"    night mode      -> {night}")
            updated.append("night_mode")

        if ps.HasField("evening_minutes_mn"):
            self._settings_override.evening_minutes_mn = ps.evening_minutes_mn
            print(f"    evening run     -> {ps.evening_minutes_mn} min")
            updated.append("evening")

        if ps.HasField("morning_minutes_mn"):
            self._settings_override.morning_minutes_mn = ps.morning_minutes_mn
            print(f"    morning run     -> {ps.morning_minutes_mn} min")
            updated.append("morning")

        if ps.HasField("night_threshold_mv"):
            self._settings_override.night_threshold_mv = ps.night_threshold_mv
            print(f"    night threshold -> {ps.night_threshold_mv / 1000:.2f} V")
            updated.append("night_thresh")

        if ps.HasField("dimming_pct"):
            self._settings_override.dimming_pct = ps.dimming_pct
            print(f"    dimming         -> {ps.dimming_pct}%")
            updated.append("dimming")

        if ps.HasField("base_dimming_pct"):
            self._settings_override.base_dimming_pct = ps.base_dimming_pct
            print(f"    base dimming    -> {ps.base_dimming_pct}%")
            updated.append("base_dim")

        if ps.HasField("dali_power_enable"):
            self._settings_override.dali_power_enable = ps.dali_power_enable
            print(f"    DALI power      -> {'on' if ps.dali_power_enable else 'off'}")
            updated.append("dali")

        if ps.HasField("alc_dimming_enable"):
            self._settings_override.alc_dimming_enable = ps.alc_dimming_enable
            print(f"    ALC dimming     -> {'on' if ps.alc_dimming_enable else 'off'}")
            updated.append("alc")

        if not updated:
            print("    (no fields changed)")

        # Re-publish full updated settings
        client.publish(
            TOPIC_SETTINGS.format(gw=self.gateway, serial=self.serial),
            self.get_settings_proto(),
            qos=1,
            retain=True,
        )

    def get_info_proto(self) -> bytes:
        msg = mppt_pb2.DeviceInfo()
        msg.serial_number = self.serial
        msg.production_date = "2024-03-15"
        msg.device_type = "Phocos CIS-N"
        msg.hw_version = self.hw
        return msg.SerializeToString()

    def get_settings_proto(self) -> bytes:
        msg = mppt_pb2.DeviceSettings()
        # Default values
        msg.serial = self.serial
        msg.timestamp.FromDatetime(datetime.now(timezone.utc))
        msg.battery_type_index = 1  # LiFePO4 Med
        msg.capacity_ah = 128
        msg.lvd_voltage_mv = 11000
        msg.lvd_mode_voltage = False
        msg.night_mode_index = 1  # D2D
        msg.evening_minutes_mn = 120
        msg.morning_minutes_mn = 60
        msg.night_threshold_mv = 11500
        msg.night_mode_dimming_index = 0
        msg.evening_minutes_dimming_mn = 0
        msg.morning_minutes_dimming_mn = 0
        msg.dimming_pct = 100
        msg.base_dimming_pct = 30
        msg.dali_power_enable = False
        msg.alc_dimming_enable = False
        if self._settings_override is not None:
            msg.CopyFrom(self._settings_override)
            msg.timestamp.FromDatetime(datetime.now(timezone.utc))
        return msg.SerializeToString()

    def get_telemetry_proto(self, zone: str) -> bytes:
        msg = mppt_pb2.Telemetry()
        msg.zone = zone
        msg.gateway_id = self.gateway
        msg.serial = self.serial
        msg.hw_version = self.hw
        msg.timestamp.FromDatetime(datetime.now(timezone.utc))

        msg.firmware_version = 17
        msg.internal_temp_c = int(self.temp_int)
        msg.external_temp_c = int(self.temp_ext)
        msg.controller_op_days = 350

        msg.battery_voltage_v = round(self.vbat, 3)
        msg.battery_soc_pct = int(self.soc)
        msg.charge_current_a = round(self.ichg, 2)
        msg.charge_power_w = int(self.ichg * self.vbat)
        msg.end_of_charge_voltage_v = round(self.vpv_target, 3)
        msg.charge_mode = self.charge_mode
        msg.is_night = False
        msg.bat_op_days = 350
        msg.energy_in_daily_wh = 450
        msg.energy_out_daily_wh = 380
        msg.energy_retained_wh = 210
        msg.battery_detected = True

        msg.load_on = True
        msg.night_mode = False
        msg.lvd_active = self.soc < 10.0
        msg.user_disconnect = False
        msg.over_current = False
        msg.load_current_a = round(self.iload, 2)
        msg.load_power_w = int(self.iload * self.vbat)

        msg.pv_voltage_v = round(self.vpv, 3)
        msg.pv_target_voltage_v = round(self.vpv_target, 3)
        msg.pv_detected = True

        msg.time_since_dusk_min = 0
        msg.average_length_min = 640

        if self.hw == 3:
            msg.led_voltage_v = round(self.led_v, 3)
            msg.led_current_a = round(self.led_i, 2)
            msg.led_power_w = int(self.led_v * self.led_i)
            msg.led_status = "Normal"
            msg.dali_active = False

        msg.fault_battery_over_voltage = self.faults["battery_over_voltage"]
        msg.fault_pv_over_voltage = self.faults["pv_over_voltage"]
        msg.fault_controller_over_temp = self.faults["controller_over_temp"]
        msg.fault_charge_over_current = self.faults["charge_over_current"]
        msg.fault_lvd_active = self.faults["lvd_active"]
        msg.fault_over_discharge_current = self.faults["over_discharge_current"]
        msg.fault_battery_over_temp = self.faults["battery_over_temp"]
        msg.fault_battery_under_temp = self.faults["battery_under_temp"]

        return msg.SerializeToString()

    def _make_datalogger_base(self, zone: str) -> mppt_pb2.DataloggerPayload:
        """Build a DataloggerPayload without log entries (caller appends them)."""
        msg = mppt_pb2.DataloggerPayload()
        msg.zone = zone
        msg.gateway_id = self.gateway
        msg.serial = self.serial
        msg.timestamp.FromDatetime(datetime.now(timezone.utc))
        msg.battery_type = "LiFePO4 - Medium Temp"
        msg.capacity_ah = 128
        msg.recorded_days = TOTAL_DAYS
        msg.days_with_lvd = _rnd_int(0, 5)
        msg.months_without_full_charge = _rnd_int(0, 2)
        msg.avg_morning_soc_pct = _rnd(20.0, 80.0, 1)
        msg.total_ah_charge = _rnd(800.0, 8000.0, 1)
        msg.total_ah_load = _rnd(700.0, 7000.0, 1)
        return msg

    @staticmethod
    def _fill_log_entry(entry: mppt_pb2.LogEntry, row: dict):
        """Populate a LogEntry proto from a row dict produced by _daily/_monthly_row."""
        entry.index = row["index"]
        entry.vbat_min_v = row["vbat_min_v"]
        entry.vbat_max_v = row["vbat_max_v"]
        entry.vpv_min_v = row["vpv_min_v"]
        entry.vpv_max_v = row["vpv_max_v"]
        entry.ah_charge = row["ah_charge"]
        entry.ah_load = row["ah_load"]
        entry.il_max_a = row["il_max_a"]
        entry.ipv_max_a = row["ipv_max_a"]
        entry.soc_pct = row["soc_pct"]
        entry.ext_temp_max_c = row["ext_temp_max_c"]
        entry.ext_temp_min_c = row["ext_temp_min_c"]
        entry.nightlength_min = row["nightlength_min"]
        entry.flag_full_charge = row.get("flag_full_charge", False)
        entry.flag_low_soc = row.get("flag_low_soc", False)


def on_connect(client, userdata, flags, reason_code, properties):
    print(f"[MQTT] Connected (rc={reason_code})")
    for dev in userdata["devices"]:
        client.publish(
            TOPIC_ONLINE.format(gw=dev.gateway, serial=dev.serial),
            "1",
            qos=1,
            retain=True,
        )
        client.publish(
            TOPIC_INFO.format(gw=dev.gateway, serial=dev.serial),
            dev.get_info_proto(),
            qos=1,
            retain=True,
        )
        client.publish(
            TOPIC_SETTINGS.format(gw=dev.gateway, serial=dev.serial),
            dev.get_settings_proto(),
            qos=1,
            retain=True,
        )
        cmd_topic = TOPIC_CMD.format(serial=dev.serial)
        client.subscribe(cmd_topic, qos=1)
        print(f"  {dev.serial}  published info+settings  subscribed {cmd_topic}")


def on_message(client, userdata, msg):
    """Handle inbound mppt::ControlCommand (binary Protobuf)."""
    cmd = mppt_pb2.ControlCommand()
    try:
        cmd.ParseFromString(msg.payload)
    except Exception as exc:
        print(f"[CMD] Proto parse error on {msg.topic}: {exc}")
        return

    dev = next((d for d in userdata["devices"] if d.serial == cmd.serial), None)
    if dev is None:
        print(f"[CMD] Unknown serial {cmd.serial!r} — ignoring")
        return

    print(f"\n[CMD] -> {cmd.serial}  (request_id={cmd.request_id})")

    ok = False
    reason = "Unknown command"

    which = cmd.WhichOneof("payload")

    if which == "set_settings":
        dev.apply_settings(cmd.set_settings, client)
        ok, reason = True, "Settings applied"

    elif which == "clear_datalogger":
        dev._burst_sent = False
        dev._new_day_sent = False
        dev._burst_daily = [dev._daily_row(d) for d in range(1, TOTAL_DAYS + 1)]
        dev._burst_monthly = [dev._monthly_row(m) for m in range(1, TOTAL_MONTHS + 1)]
        print(f"  [{dev.serial}] datalogger cleared — will re-burst next publish")
        ok, reason = True, "Datalogger cleared"

    elif which == "switch_load":
        print(f"  [{dev.serial}] SWITCH_LOAD on={cmd.switch_load.on} - not implemented")
        ok, reason = False, "SWITCH_LOAD not implemented"

    else:
        print(f"  [{dev.serial}] empty or unknown oneof payload")

    ack = mppt_pb2.CommandAck()
    ack.serial = cmd.serial
    ack.request_id = cmd.request_id
    ack.ok = ok
    ack.reason = reason
    ack.timestamp.FromDatetime(datetime.now(timezone.utc))

    ack_topic = TOPIC_ACK.format(gw=dev.gateway, serial=dev.serial)
    client.publish(ack_topic, ack.SerializeToString(), qos=1)
    status = "OK" if ok else "NOT OK"
    print(f"  [{dev.serial}] {status} ACK -> {ack_topic}  ({reason})\n")


def publish_loop(client, dev: SimDevice, lock: threading.Lock):
    topic_state = TOPIC_STATE.format(gw=dev.gateway, serial=dev.serial)
    topic_datalog = TOPIC_DATALOG.format(gw=dev.gateway, serial=dev.serial)

    while True:
        dev.step()
        now = time.time()
        should_dl = False
        dl_msg = dev._make_datalogger_base("default")

        if not dev._burst_sent:
            for row in dev._burst_daily:
                dev._fill_log_entry(dl_msg.daily_logs.add(), row)
            for row in dev._burst_monthly:
                dev._fill_log_entry(dl_msg.monthly_logs.add(), row)
            dev._burst_sent = True
            should_dl = True
            print(
                f"  [{dev.serial}] * burst  {len(dev._burst_daily)}d + {len(dev._burst_monthly)}m"
            )

        elif not dev._new_day_sent and (now - dev._startup_time) > NEW_DAY_DELAY_S:
            dev._fill_log_entry(dl_msg.daily_logs.add(), dev._daily_row(30))
            dev._new_day_sent = True
            should_dl = True
            print(f"  [{dev.serial}] * day-30 re-injected")

        with lock:
            client.publish(topic_state, dev.get_telemetry_proto("default"), qos=0)
            if should_dl:
                client.publish(
                    topic_datalog, dl_msg.SerializeToString(), qos=1, retain=True
                )

        print(
            f"  -> {dev.serial:<10} {dev.charge_mode:<14}"
            f" SOC={dev.soc:5.1f}%  Vbat={dev.vbat:5.3f}V"
        )
        time.sleep(PUBLISH_INTERVAL_S)


def main():
    devices = [
        SimDevice("0503-18", hw=3, gateway="sim-gw", start_mode="Float"),
        SimDevice("0166-18", hw=3, gateway="sim-gw", start_mode="MPP Tracking"),
        SimDevice("0977-08", hw=2, gateway="sim-gw", start_mode="Boost"),
    ]

    client = mqtt.Client(
        CallbackAPIVersion.VERSION2,
        userdata={"devices": devices},
    )
    client.on_connect = on_connect
    client.on_message = on_message

    print(f"[MQTT] Connecting to {MQTT_HOST}:{MQTT_PORT} ...")
    client.connect(MQTT_HOST, MQTT_PORT, keepalive=60)
    client.loop_start()
    time.sleep(0.6)

    print(f"\nSimulator running - publishing every {PUBLISH_INTERVAL_S}s")
    print(
        f"First message per device: all {TOTAL_DAYS} days + {TOTAL_MONTHS} months of history"
    )
    print(f"Day-30 re-injected after {NEW_DAY_DELAY_S}s\n")

    lock = threading.Lock()
    threads = [
        threading.Thread(target=publish_loop, args=(client, dev, lock), daemon=True)
        for dev in devices
    ]
    for t in threads:
        t.start()

    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print("\n[Stopped by user]")
    finally:
        for dev in devices:
            client.publish(
                TOPIC_ONLINE.format(gw=dev.gateway, serial=dev.serial),
                "0",
                qos=1,
                retain=True,
            )
        time.sleep(0.3)
        client.loop_stop()
        client.disconnect()


if __name__ == "__main__":
    main()
