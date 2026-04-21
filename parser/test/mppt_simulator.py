#!/usr/bin/env python3
import math
import os
import random
import sys
import threading
import time

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
TOPIC_FAULTS = "mppt/default/{gw}/{serial}/faults"
TOPIC_ONLINE = "mppt/default/{gw}/{serial}/online"
TOPIC_DATALOG_SUMMARY = "mppt/default/{gw}/{serial}/datalog/summary"
TOPIC_DATALOG_DAILY = "mppt/default/{gw}/{serial}/datalog/daily"
TOPIC_DATALOG_MONTHLY = "mppt/default/{gw}/{serial}/datalog/monthly"
TOPIC_INFO = "mppt/default/{gw}/{serial}/info"
TOPIC_SETTINGS = "mppt/default/{gw}/{serial}/settings"
TOPIC_CMD = "mppt/+/+/{serial}/cmd"  # subscribe pattern per device
TOPIC_ACK = "mppt/default/{gw}/{serial}/ack"

TOTAL_DAYS = 30
TOTAL_MONTHS = 12

# Map human-readable names to Proto Enums
CHARGE_MODE_MAP = {
    "MPP Tracking": mppt_pb2.CHARGE_MODE_BOOST,
    "Boost": mppt_pb2.CHARGE_MODE_BOOST,
    "Float": mppt_pb2.CHARGE_MODE_FLOAT,
    "Equalize": mppt_pb2.CHARGE_MODE_EQUALIZATION,
    "Disabled": mppt_pb2.CHARGE_MODE_DISABLED,
}

FAULT_BITS = {
    "battery_over_voltage": 0x01,
    "pv_over_voltage": 0x02,
    "controller_over_temp": 0x04,
    "charge_over_current": 0x08,
    "lvd_active": 0x10,
    "over_discharge_current": 0x20,
    "battery_over_temp": 0x40,
    "battery_under_temp": 0x80,
}


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

        # Parse numeric identifier from serial (e.g. "0503-18" -> 50318)
        try:
            self.device_id = int(serial.replace("-", ""))
        except ValueError:
            self.device_id = 12345

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
        self._last_fault_mask = 0

        self._last_t = time.time()
        self._startup_time = time.time()
        self._new_day_sent = False
        self._settings_override: mppt_pb2.DeviceSettings | None = None

        self._burst_daily = [self._daily_row(d) for d in range(1, TOTAL_DAYS + 1)]
        self._burst_monthly = [self._monthly_row(m) for m in range(1, TOTAL_MONTHS + 1)]
        self._burst_sent = False

        # Incremental delta tracking
        self._delta_daily: dict[int, dict] = {}
        self._delta_monthly: dict[int, dict] = {}

    @staticmethod
    def _jitter(v: float, pct: float = 0.02) -> float:
        return v * (1.0 + random.uniform(-pct, pct))

    @staticmethod
    def _sine(t: float, period: float, lo: float, hi: float) -> float:
        return lo + (hi - lo) * (0.5 + 0.5 * math.sin(2 * math.pi * t / period))

    def get_fault_mask(self) -> int:
        mask = 0
        for name, bit in FAULT_BITS.items():
            if self.faults.get(name):
                mask |= bit
        return mask

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

        # Randomly toggle some faults using the bitmasks
        for name, bit in FAULT_BITS.items():
            if random.random() < 0.05:
                self.faults[name] = not self.faults[name]

        # Override specific faults
        self.faults["lvd_active"] = self.soc < 5.0
        self.faults["controller_over_temp"] = self.temp_int > 45.0
        self.faults["battery_over_temp"] = self.temp_int > 50.0
        self.faults["battery_under_temp"] = self.temp_ext < 12.0

    def _daily_row(self, day: int) -> dict:
        base_vbat = _rnd(12.0, 14.2, 3)
        base_vpv = _rnd(8.0, 20.0, 3)
        return {
            "index": day,
            "vbat_min_mv": int(round(base_vbat - _rnd(0.5, 1.5, 3), 3) * 1000),
            "vbat_max_mv": int(round(base_vbat + _rnd(0.2, 0.8, 3), 3) * 1000),
            "vpv_min_mv": int(round(max(0.0, base_vpv - _rnd(5.0, 10.0, 3)), 3) * 1000),
            "vpv_max_mv": int(base_vpv * 1000),
            "ah_charge_mah": int(_rnd(10.0, 70.0, 1)),
            "ah_load_mah": int(_rnd(8.0, 60.0, 1)),
            "il_max_ma10": int(_rnd(0.5, 12.0, 2) * 10),
            "ipv_max_ma10": int(_rnd(1.0, 10.0, 2) * 10),
            "soc_pct": int(_rnd_int(5, 100)),
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
            "vbat_min_mv": int(_rnd(11.2, 12.5, 3) * 1000),
            "vbat_max_mv": int(_rnd(13.5, 14.2, 3) * 1000),
            "vpv_min_mv": int(_rnd(3.0, 7.0, 3) * 1000),
            "vpv_max_mv": int(_rnd(17.0, 22.0, 3) * 1000),
            "ah_charge_mah": int(round(200.0 + season * 700.0 + _rnd(-50, 50, 1), 1)),
            "ah_load_mah": int(round(150.0 + season * 500.0 + _rnd(-30, 30, 1), 1)),
            "il_max_ma10": int(_rnd(1.0, 4.0, 2) * 10),
            "ipv_max_ma10": int(_rnd(2.0, 8.0, 2) * 10),
            "soc_pct": int(_rnd_int(20, 90)),
            "ext_temp_max_c": _rnd_int(20, 50),
            "ext_temp_min_c": _rnd_int(0, 20),
            "nightlength_min": _rnd_int(540, 720),
            "flag_full_charge": month % 4 == 0,
            "flag_low_soc": random.random() > 0.8,
        }

    def apply_settings(self, ps: mppt_pb2.DeviceSettings, client: mqtt.Client):
        """Apply partial DeviceSettings safely using HasField() for optional fields."""
        print(f"  [{self.serial}] Received partial settings update:")

        if self._settings_override is None:
            self._settings_override = mppt_pb2.DeviceSettings()
            # Load current defaults
            tmp_bytes = self.get_settings_proto()
            self._settings_override.ParseFromString(tmp_bytes)

        updated = []

        if ps.HasField("battery_type"):
            self._settings_override.battery_type = ps.battery_type
            updated.append(f"battery_type={ps.battery_type}")

        if ps.HasField("capacity_ah"):
            self._settings_override.capacity_ah = ps.capacity_ah
            updated.append(f"capacity={ps.capacity_ah}Ah")

        if ps.HasField("lvd_voltage_mv"):
            self._settings_override.lvd_voltage_mv = ps.lvd_voltage_mv
            updated.append(f"lvd_mv={ps.lvd_voltage_mv}")

        if ps.HasField("lvd_mode"):
            self._settings_override.lvd_mode = ps.lvd_mode
            updated.append(f"lvd_mode={ps.lvd_mode}")

        if ps.HasField("night_mode"):
            self._settings_override.night_mode = ps.night_mode
            updated.append(f"night_mode={ps.night_mode}")

        if ps.HasField("evening_minutes"):
            self._settings_override.evening_minutes = ps.evening_minutes
            updated.append(f"evening={ps.evening_minutes}m")

        if ps.HasField("morning_minutes"):
            self._settings_override.morning_minutes = ps.morning_minutes
            updated.append(f"morning={ps.morning_minutes}m")

        if ps.HasField("night_threshold_mv"):
            self._settings_override.night_threshold_mv = ps.night_threshold_mv
            updated.append(f"threshold_mv={ps.night_threshold_mv}")

        if ps.HasField("dimming_mode"):
            self._settings_override.dimming_mode = ps.dimming_mode
            updated.append(f"dimming_mode={ps.dimming_mode}")

        if ps.HasField("evening_minutes_dimming"):
            self._settings_override.evening_minutes_dimming = ps.evening_minutes_dimming
            updated.append(f"eve_dim={ps.evening_minutes_dimming}m")

        if ps.HasField("morning_minutes_dimming"):
            self._settings_override.morning_minutes_dimming = ps.morning_minutes_dimming
            updated.append(f"morn_dim={ps.morning_minutes_dimming}m")

        if ps.HasField("dimming_pct"):
            self._settings_override.dimming_pct = ps.dimming_pct
            updated.append(f"dimming={ps.dimming_pct}%")

        if ps.HasField("base_dimming_pct"):
            self._settings_override.base_dimming_pct = ps.base_dimming_pct
            updated.append(f"base_dim={ps.base_dimming_pct}%")

        if ps.HasField("advanced_flags"):
            self._settings_override.advanced_flags = ps.advanced_flags
            updated.append(f"adv_flags=0x{ps.advanced_flags:02X}")

        if not updated:
            print("    (no fields changed)")
        else:
            print(f"    Updated: {', '.join(updated)}")

        # Re-publish full updated settings
        client.publish(
            TOPIC_SETTINGS.format(gw=self.gateway, serial=self.serial),
            self.get_settings_proto(),
            qos=1,
            retain=True,
        )

    def get_info_proto(self) -> bytes:
        msg = mppt_pb2.DeviceInfo()
        msg.production_date = "2024-03-15"
        msg.hw_version = self.hw
        msg.firmware_version = 17
        msg.device_type = "MPPT"
        msg.equalization_voltage_mv = 14800
        msg.boost_voltage_mv = 14400
        msg.float_voltage_mv = 13800
        msg.temp_comp_mv_per_c = -30
        msg.published_at = int(time.time())
        return msg.SerializeToString()

    def get_settings_proto(self) -> bytes:
        msg = mppt_pb2.DeviceSettings()
        msg.timestamp = int(time.time())
        msg.battery_type = mppt_pb2.BATTERY_LFP_MEDIUM_TEMP
        msg.capacity_ah = 128
        msg.lvd_voltage_mv = 11000
        msg.lvd_mode = mppt_pb2.LVD_MODE_VOLTAGE
        msg.night_mode = mppt_pb2.NIGHT_MODE_D2D
        msg.evening_minutes = 120
        msg.morning_minutes = 60
        msg.night_threshold_mv = 11500
        msg.dimming_mode = mppt_pb2.NIGHT_MODE_ALWAYS_ON
        msg.evening_minutes_dimming = 0
        msg.morning_minutes_dimming = 0
        msg.dimming_pct = 100
        msg.base_dimming_pct = 30
        msg.advanced_flags = 0x00

        if self._settings_override is not None:
            ts = msg.timestamp
            msg.CopyFrom(self._settings_override)
            msg.timestamp = ts

        return msg.SerializeToString()

    def get_telemetry_proto(self) -> bytes:
        msg = mppt_pb2.Telemetry()
        msg.timestamp = int(time.time())
        msg.internal_temp_c = int(self.temp_int)
        msg.external_temp_c = int(self.temp_ext)
        msg.controller_op_days = 350
        msg.battery_voltage_mv = int(self.vbat)
        msg.battery_soc_pct = int(self.soc)
        msg.charge_current_ma10 = int(self.ichg * 100)
        msg.charge_power_w = int(self.ichg * self.vbat)
        msg.end_of_charge_voltage_mv = int(self.vpv_target * 1000)
        msg.charge_mode = CHARGE_MODE_MAP.get(
            self.charge_mode, mppt_pb2.CHARGE_MODE_DISABLED
        )
        msg.bat_op_days = 350
        msg.energy_in_daily_wh = 450
        msg.energy_out_daily_wh = 380
        msg.energy_retained_wh = 210
        msg.pv_voltage_mv = int(self.vpv * 1000)
        msg.pv_target_voltage_mv = int(self.vpv_target * 1000)
        msg.load_current_ma10 = int(self.iload * 100)
        msg.load_power_w = int(self.iload * self.vbat)
        msg.time_since_dusk_min = 0
        msg.average_length_min = 640
        msg.fault_mask = self.get_fault_mask()
        
        flags = 0
        flags |= 1 << 0
        if self.charge_mode == "Float" and random.random() > 0.9:
            flags |= 1 << 1
        flags |= 1 << 2
        if self.soc < 10.0:
            flags |= 1 << 4
        flags |= 1 << 7
        msg.flags = flags
        if self.hw == 3:
            msg.led_voltage_mv = int(self.led_v * 1000)
            msg.led_current_ma10 = int(self.led_i * 1000 / 10)
            msg.led_power_w = int(self.led_v * self.led_i)
            msg.led_status = mppt_pb2.LED_NORMAL
        return msg.SerializeToString()

    def get_fault_status_proto(self) -> bytes:
        msg = mppt_pb2.FaultStatus()
        msg.fault_mask = self.get_fault_mask()
        msg.load_state_mask = 0x01 if self.soc < 10.0 else 0x00
        msg.charge_state_mask = 0x02 if self.charge_mode == "Float" else 0x01
        msg.timestamp = int(time.time())
        return msg.SerializeToString()

    def _make_datalogger_base(self) -> mppt_pb2.DataloggerPayload:
        msg = mppt_pb2.DataloggerPayload()
        msg.timestamp = int(time.time())
        msg.recorded_days = TOTAL_DAYS
        msg.days_with_lvd = _rnd_int(0, 5)
        msg.months_without_full_charge = _rnd_int(0, 2)
        msg.avg_morning_soc_pct = int(_rnd(20.0, 80.0, 1))
        msg.total_ah_charge_mah = int(_rnd(800.0, 8000.0, 1))
        msg.total_ah_load_mah = int(_rnd(700.0, 7000.0, 1))
        return msg

    @staticmethod
    def _fill_log_entry(entry: mppt_pb2.LogEntry, row: dict):
        entry.index = row["index"]
        entry.vbat_min_mv = row["vbat_min_mv"]
        entry.vbat_max_mv = row["vbat_max_mv"]
        entry.vpv_min_mv = row["vpv_min_mv"]
        entry.vpv_max_mv = row["vpv_max_mv"]
        entry.ah_charge_mah = row["ah_charge_mah"]
        entry.ah_load_mah = row["ah_load_mah"]
        entry.il_max_ma10 = row["il_max_ma10"]
        entry.ipv_max_ma10 = row["ipv_max_ma10"]
        entry.soc_pct = row["soc_pct"]
        entry.ext_temp_max_c = row["ext_temp_max_c"]
        entry.ext_temp_min_c = row["ext_temp_min_c"]
        entry.nightlength_min = row["nightlength_min"]
        sf = 0
        if row.get("flag_low_soc"):
            sf |= 1 << 5
        if row.get("flag_full_charge"):
            sf |= 1 << 1
        entry.state_flags = sf


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
    parts = msg.topic.split("/")
    if len(parts) < 4:
        return
    serial = parts[3]
    dev = next((d for d in userdata["devices"] if d.serial == serial), None)
    if dev is None:
        return
    cmd = mppt_pb2.ControlCommand()
    try:
        cmd.ParseFromString(msg.payload)
    except Exception as exc:
        print(f"[CMD] Proto parse error: {exc}")
        return
    print(f"\n[CMD] -> {serial}  (request_id={cmd.request_id})")
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
        ok, reason = True, "Datalogger cleared"
    ack = mppt_pb2.CommandAck()
    ack.request_id = cmd.request_id
    ack.ok = ok
    ack.reason = reason
    ack.timestamp = int(time.time())
    ack_topic = TOPIC_ACK.format(gw=dev.gateway, serial=dev.serial)
    client.publish(ack_topic, ack.SerializeToString(), qos=1)
    print(f"  [{dev.serial}] ACK -> {ack_topic} ({reason})\n")


def publish_loop(client, dev: SimDevice, lock: threading.Lock):
    topic_state = TOPIC_STATE.format(gw=dev.gateway, serial=dev.serial)
    topic_faults = TOPIC_FAULTS.format(gw=dev.gateway, serial=dev.serial)
    topic_summary = TOPIC_DATALOG_SUMMARY.format(gw=dev.gateway, serial=dev.serial)
    topic_daily = TOPIC_DATALOG_DAILY.format(gw=dev.gateway, serial=dev.serial)
    topic_monthly = TOPIC_DATALOG_MONTHLY.format(gw=dev.gateway, serial=dev.serial)
    while True:
        dev.step()
        now = time.time()
        should_dl_summary = False
        should_dl_daily = False
        should_dl_monthly = False
        dl_summary = dev._make_datalogger_base()
        dl_daily = dev._make_datalogger_base()
        dl_monthly = dev._make_datalogger_base()
        if not dev._burst_sent:
            for row in dev._burst_daily:
                dev._fill_log_entry(dl_daily.daily_logs.add(), row)
            for row in dev._burst_monthly:
                dev._fill_log_entry(dl_monthly.monthly_logs.add(), row)
            dev._burst_sent = True
            should_dl_summary = should_dl_daily = should_dl_monthly = True
        elif not dev._new_day_sent and (now - dev._startup_time) > NEW_DAY_DELAY_S:
            dev._fill_log_entry(dl_daily.daily_logs.add(), dev._daily_row(30))
            dev._new_day_sent = True
            should_dl_summary = should_dl_daily = True
        with lock:
            client.publish(topic_state, dev.get_telemetry_proto(), qos=0)
            current_mask = dev.get_fault_mask()
            if current_mask != dev._last_fault_mask or not dev._burst_sent:
                client.publish(
                    topic_faults, dev.get_fault_status_proto(), qos=1, retain=True
                )
                dev._last_fault_mask = current_mask
            if should_dl_summary:
                client.publish(
                    topic_summary, dl_summary.SerializeToString(), qos=1, retain=True
                )
            if should_dl_daily:
                client.publish(
                    topic_daily, dl_daily.SerializeToString(), qos=1, retain=True
                )
            if should_dl_monthly:
                client.publish(
                    topic_monthly, dl_monthly.SerializeToString(), qos=1, retain=True
                )
        print(
            f"  -> {dev.serial:<10} {dev.charge_mode:<14} SOC={dev.soc:5.1f}%  Vbat={dev.vbat:5.3f}V"
        )
        time.sleep(PUBLISH_INTERVAL_S)


def main():
    devices = [
        SimDevice("0503-18", hw=3, gateway="sim-gw", start_mode="Float"),
        SimDevice("0166-18", hw=3, gateway="sim-gw", start_mode="MPP Tracking"),
        SimDevice("0977-08", hw=2, gateway="sim-gw", start_mode="Boost"),
    ]
    client = mqtt.Client(CallbackAPIVersion.VERSION2, userdata={"devices": devices})
    client.on_connect = on_connect
    client.on_message = on_message
    print(f"[MQTT] Connecting to {MQTT_HOST}:{MQTT_PORT} ...")
    client.connect(MQTT_HOST, MQTT_PORT, keepalive=60)
    client.loop_start()
    time.sleep(0.6)
    print(f"\nSimulator running - publishing every {PUBLISH_INTERVAL_S}s")
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
