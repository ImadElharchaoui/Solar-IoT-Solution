#!/usr/bin/env python3
import json
import math
import os
import random
import threading
import time
from datetime import datetime, timezone

import paho.mqtt.client as mqtt

MQTT_HOST          = "localhost"
MQTT_PORT          = 1883
PUBLISH_INTERVAL_S = 5   # live telemetry cadence
NEW_DAY_DELAY_S    = 60  # seconds before a fresh day-30 is re-injected

TOPIC_STATE   = "mppt/default/{gw}/{serial}/state"
TOPIC_ONLINE  = "mppt/default/{gw}/{serial}/online"
TOPIC_DATALOG = "mppt/default/{gw}/{serial}/datalog"

TOTAL_DAYS   = 30
TOTAL_MONTHS = 12


def rnd(lo, hi, dec=2):
    return round(lo + random.random() * (hi - lo), dec)

def rnd_int(lo, hi):
    return random.randint(lo, hi)

MODE_SCHEDULE = [
    ("MPP Tracking", 120),
    ("Boost",         60),
    ("Float",        180),
]

class SimDevice:
    def __init__(self, serial, hw, gateway, start_mode):
        self.serial  = serial
        self.hw      = hw
        self.gateway = gateway

        self._mode_idx   = next(i for i, (m, _) in enumerate(MODE_SCHEDULE) if m == start_mode)
        self._mode_timer = 0.0
        self.charge_mode = start_mode

        self.soc        = rnd(40.0, 90.0, 1)
        self.vbat       = rnd(12.8, 13.8, 3)
        self.vpv        = rnd(15.0, 20.0, 3)
        self.vpv_target = rnd(14.5, 16.5, 3)
        self.ichg       = rnd(1.0, 5.0, 2)
        self.iload      = rnd(0.5, 3.0, 2)
        self.temp_int   = rnd(22.0, 35.0, 1)
        self.temp_ext   = rnd(18.0, 32.0, 1)
        self.led_v      = rnd(11.5, 13.5, 3) if hw == 3 else 0.0
        self.led_i      = rnd(0.01, 0.15, 2) if hw == 3 else 0.0

        self.faults = {
            "battery_over_voltage":   False,
            "pv_over_voltage":        False,
            "controller_over_temp":   False,
            "charge_over_current":    False,
            "lvd_active":             False,
            "over_discharge_current": False,
            "battery_over_temp":      False,
            "battery_under_temp":     False,
        }

        self._last_t        = time.time()
        self._startup_time  = time.time()
        self._publish_count = 0
        self._new_day_sent  = False

        # Pre-generate the full history burst, sent in the FIRST message
        self._burst_daily   = [self._daily_row(d)   for d in range(1, TOTAL_DAYS   + 1)]
        self._burst_monthly = [self._monthly_row(m) for m in range(1, TOTAL_MONTHS + 1)]
        self._burst_sent    = False

    @staticmethod
    def _jitter(v, pct=0.02):
        return v * (1.0 + random.uniform(-pct, pct))

    @staticmethod
    def _sine(t, period, lo, hi):
        return lo + (hi - lo) * (0.5 + 0.5 * math.sin(2 * math.pi * t / period))

    def step(self):
        now = time.time()
        dt  = now - self._last_t
        self._last_t = now

        self._mode_timer += dt
        _, dur = MODE_SCHEDULE[self._mode_idx]
        if self._mode_timer >= dur:
            self._mode_timer = 0.0
            self._mode_idx   = (self._mode_idx + 1) % len(MODE_SCHEDULE)
        self.charge_mode, _ = MODE_SCHEDULE[self._mode_idx]

        if self.charge_mode == "MPP Tracking":
            vpv_base  = self._sine(now, 300, 16.0, 20.0)
            ichg_base = self._sine(now, 240,  4.0, 10.0)
            self.vpv_target = self._jitter(15.5, 0.03)
        elif self.charge_mode == "Boost":
            vpv_base  = self._sine(now, 300, 14.0, 17.0)
            ichg_base = self._sine(now, 240,  2.0,  6.0)
            self.vpv_target = self._jitter(14.4, 0.02)
        else:
            vpv_base  = self._sine(now, 300,  9.0, 14.0)
            ichg_base = self._sine(now, 240,  0.1,  1.5)
            self.vpv_target = self._jitter(13.8, 0.02)

        self.vpv  = self._jitter(vpv_base,  0.02)
        self.ichg = max(0.0, self._jitter(ichg_base, 0.05))
        net_ah    = (self.ichg - self.iload) * dt / 3600.0
        self.soc  = max(0.0, min(100.0, self.soc + net_ah / 1.28))
        self.vbat = 11.0 + (self.soc / 100.0) * 3.0 + self._jitter(0.0, 0.005)

        if self.charge_mode == "Boost":
            self.vbat = min(14.4, self.vbat + self._jitter(0.3, 0.05))
        elif self.charge_mode == "Float":
            floor     = 13.6 if self.soc > 80 else self.vbat
            self.vbat = min(13.8, max(self.vbat, floor))

        self.iload    = max(0.5, min(3.0, self.iload + random.uniform(-0.1, 0.1)))
        self.temp_int = self._jitter(self._sine(now, 600, 22.0, 42.0), 0.01)
        self.temp_ext = self._jitter(self._sine(now, 900, 18.0, 38.0), 0.01)

        if self.hw == 3:
            night      = (int(now / 60) % 20) < 12
            self.led_v = self._jitter(12.0, 0.02) if night else self._jitter(0.2, 0.10)
            self.led_i = self._jitter(0.85, 0.05) if night else 0.0

        for fault in list(self.faults):
            self.faults[fault] = random.random() < 0.15
        if self.soc < 5.0:
            self.faults["lvd_active"] = True
        self.faults["controller_over_temp"] = self.temp_int > 45.0
        self.faults["battery_over_temp"]  = self.temp_int > 50.0
        self.faults["battery_under_temp"] = self.temp_ext < 12.0

    def _daily_row(self, day):
        base_vbat = rnd(12.0, 14.2, 3)
        base_vpv  = rnd( 8.0, 20.0, 3)
        return {
            "day":             day,
            "vbat_min_v":      round(base_vbat - rnd(0.5, 1.5, 3), 3),
            "vbat_max_v":      round(base_vbat + rnd(0.2, 0.8, 3), 3),
            "vpv_min_v":       round(max(0.0, base_vpv - rnd(5.0, 10.0, 3)), 3),
            "vpv_max_v":       base_vpv,
            "ah_charge":       rnd(10.0, 70.0, 1),
            "ah_load":         rnd( 8.0, 60.0, 1),
            "il_max_a":        rnd( 0.5, 12.0, 2),
            "ipv_max_a":       rnd( 1.0, 10.0, 2),
            "soc_pct":         float(rnd_int(5, 100)),
            "ext_temp_max_c":  float(rnd_int(18, 45)),
            "ext_temp_min_c":  float(rnd_int( 5, 20)),
            "nightlength_min": float(rnd_int(540, 800)),
            "state":           rnd_int(0, 3),
            "flags": {
                "ld":    False,
                "fcb":   day % 3 == 0,
                "pvoc":  False,
                "loc":   False,
                "bov":   False,
                "lsoc":  random.random() > 0.8,
                "topvo": False,
                "topvl": False,
                "tolo":  False,
            },
        }

    def _monthly_row(self, month):
        # Simulate seasonal variation: months 5-9 (summer) have more charge
        season = 0.5 + 0.5 * math.sin(math.pi * (month - 1) / 11)
        return {
            "month":           month,
            "vbat_min_v":      rnd(11.2, 12.5, 3),
            "vbat_max_v":      rnd(13.5, 14.2, 3),
            "vpv_min_v":       rnd( 3.0,  7.0, 3),
            "vpv_max_v":       rnd(17.0, 22.0, 3),
            "ah_charge":       round(200.0 + season * 700.0 + rnd(-50, 50, 1), 1),
            "ah_load":         round(150.0 + season * 500.0 + rnd(-30, 30, 1), 1),
            "il_max_a":        rnd(  1.0,   4.0, 2),
            "ipv_max_a":       rnd(  2.0,   8.0, 2),
            "soc_pct":         float(rnd_int(20, 90)),
            "ext_temp_max_c":  float(rnd_int(20, 50)),
            "ext_temp_min_c":  float(rnd_int( 0, 20)),
            "nightlength_min": float(rnd_int(540, 720)),
        }

    def get_payloads(self):
        chargeA = round(self.ichg, 2)
        loadA   = round(self.iload, 2)
        v       = round(self.vbat, 3)
        ledV    = round(self.led_v, 3) if self.hw == 3 else 0
        ledA    = round(self.led_i, 2) if self.hw == 3 else 0
        soc     = round(self.soc, 1)

        self._publish_count += 1
        now = time.time()

        should_publish_datalog = False
        # First publish: send ALL history at once
        if not self._burst_sent:
            daily   = list(self._burst_daily)    # all 30 days
            monthly = list(self._burst_monthly)  # all 12 months
            self._burst_sent = True
            should_publish_datalog = True
            print(f"  [{self.serial}] * Initial burst: {len(daily)} days + {len(monthly)} months")

        # Subsequent: no datalogger data unless it's a new-day injection
        elif not self._new_day_sent and (now - self._startup_time) > NEW_DAY_DELAY_S:
            daily   = [self._daily_row(30)]
            monthly = []
            self._new_day_sent = True
            should_publish_datalog = True
            print(f"  [{self.serial}] * New day-30 re-injected")

        else:
            daily   = []
            monthly = []

        datalog_out = {
            "recorded_days":              TOTAL_DAYS,
            "days_with_lvd":              rnd_int(0, 5),
            "months_without_full_charge": rnd_int(0, 2),
            "avg_morning_soc_pct":        rnd(20.0, 80.0, 1),
            "total_ah_charge":            rnd(800.0, 8000.0, 1),
            "total_ah_load":              rnd(700.0, 7000.0, 1),
            "daily":   daily,
            "monthly": monthly,
        }

        telemetry = {
            "general": {
                "type":               f"Solar Smart Controller V{self.hw}",
                "serial_number":      self.serial,
                "hw_version":         self.hw,
                "firmware_version":   17,
                "internal_temp_c":    round(self.temp_int, 1),
                "external_temp_c":    round(self.temp_ext, 1),
                "controller_op_days": rnd_int(300, 400),
                "production_date":    "2025-06-13",
                "timestamp":          datetime.now(timezone.utc).strftime("%H:%M:%S"),
            },
            "battery": {
                "type":                    "LiFePO4 - Medium Temp",
                "voltage_v":               v,
                "soc_pct":                 soc,
                "charge_current_a":        chargeA,
                "charge_power_w":          round(chargeA * v),
                "end_of_charge_voltage_v": round(self.vpv_target, 3),
                "charge_mode":             self.charge_mode,
                "is_night":                False,
                "operation_days":          rnd_int(300, 400),
                "energy_in_daily_wh":      rnd_int(0, 600),
                "energy_out_daily_wh":     rnd_int(200, 500),
                "energy_retained_wh":      rnd_int(100, 300),
                "detected":                True,
            },
            "load": {
                "load_on":         True,
                "night_mode":      False,
                "lvd_active":      soc < 10.0,
                "user_disconnect": False,
                "over_current":    False,
                "current_a":       loadA,
                "power_w":         round(loadA * v),
            },
            "pv": {
                "voltage_v":        round(self.vpv, 3),
                "target_voltage_v": round(self.vpv_target, 3),
                "detected":         True,
            },
            "night": {
                "time_since_dusk_min": 0,
                "average_length_min":  rnd_int(580, 680),
            },
            "led": {
                "voltage_v":   ledV,
                "current_a":   ledA,
                "power_w":     round(ledV * ledA, 1) if ledV and ledA else 0,
                "status":      "Normal",
                "dali_active": False,
            },
            "faults": dict(self.faults),
        }

        datalogger = {
            "timestamp":     datetime.now(timezone.utc).strftime("%H:%M:%S"),
            "serial_number": self.serial,
            "eeprom": {
                "battery_type": "LiFePO4 - Medium Temp",
                "capacity_ah":  128,
            },
            "datalogger": datalog_out,
        }

        return telemetry, datalogger if should_publish_datalog else None, should_publish_datalog


# MQTT
def on_connect(client, userdata, flags, reason_code, properties):
    print(f"[MQTT] Connected (rc={reason_code})")
    for dev in userdata["devices"]:
        topic = TOPIC_ONLINE.format(gw=dev.gateway, serial=dev.serial)
        client.publish(topic, "1", qos=1, retain=True)
        print(f"  online: {dev.serial}")


def publish_loop(client, dev, lock):
    topic_state   = TOPIC_STATE.format(gw=dev.gateway, serial=dev.serial)
    topic_datalog = TOPIC_DATALOG.format(gw=dev.gateway, serial=dev.serial)
    
    while True:
        dev.step()
        tele, dl, should_dl = dev.get_payloads()
        
        with lock:
            client.publish(topic_state, json.dumps(tele), qos=0)
            if should_dl and dl is not None:
                client.publish(topic_datalog, json.dumps(dl), qos=1, retain=True)

        dc  = len(dl["datalogger"]["daily"])   if dl else 0
        mc  = len(dl["datalogger"]["monthly"]) if dl else 0
        sfx = (f"  {dc}d" if dc else "") + (f"  {mc}m" if mc else "")
        print(f"  -> {dev.serial:<10} {dev.charge_mode:<14}"
              f" SOC={dev.soc:5.1f}%  Vbat={dev.vbat:5.3f}V{sfx}")
        time.sleep(PUBLISH_INTERVAL_S)


def main():
    devices = [
        SimDevice("0503-18", hw=3, gateway="sim-gw", start_mode="Float"),
        SimDevice("0166-18", hw=3, gateway="sim-gw", start_mode="MPP Tracking"),
        SimDevice("0977-08", hw=2, gateway="sim-gw", start_mode="Boost"),
    ]

    client = mqtt.Client(
        mqtt.CallbackAPIVersion.VERSION2,
        userdata={"devices": devices},
    )
    client.on_connect = on_connect

    print(f"[MQTT] Connecting to {MQTT_HOST}:{MQTT_PORT} ...")
    client.connect(MQTT_HOST, MQTT_PORT, keepalive=60)
    client.loop_start()
    time.sleep(0.6)

    print(f"\nSimulator running - publishing every {PUBLISH_INTERVAL_S}s")
    print(f"First message per device: all {TOTAL_DAYS} days + {TOTAL_MONTHS} months of history")
    print(f"Day-30 re-injected after {NEW_DAY_DELAY_S}s\n")

    lock    = threading.Lock()
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
            topic = TOPIC_ONLINE.format(gw=dev.gateway, serial=dev.serial)
            client.publish(topic, "0", qos=1, retain=True)
        time.sleep(0.3)
        client.loop_stop()
        client.disconnect()


if __name__ == "__main__":
    main()
