# MPPT Parser and Live Monitor

C++ tools for parsing and monitoring MPPT solar charge controllers over RS485 and local log files, intended for dev/testing environments.

## Tools

| Binary | Description |
|---|---|
| `mppt_parser` | Parse a local log file and print telemetry, EEPROM config, datalogger records, and settings to stdout. |
| `mppt_live` | Live RS485 poller, polls telemetry and EEPROM each cycle; can also read and write device settings in the same run, publishes to MQTT. |
| `mppt_mqtt` | Parse a local log file and publish its data to an MQTT broker. |

## Prerequisites

- C++17 compiler (`g++`)
- `make`
- `paho-mqtt-cpp` - required for `mppt_live` and `mppt_mqtt`
- `nlohmann/json` - fetched automatically as `include/json.hpp` on first build
- `libprotobuf-dev` and `protobuf-compiler` - for message serialization
- `python3-protobuf` - for Python-based monitoring tools

## Building

```bash
make release
make debug
make clean
```

## Usage

### mppt_parser - parse a local log file

```bash
./build/mppt_parser data_log.txt [--json]
```

Reads Space (` `), EEPROM (`!`), and Settings (`"`) lines from the file, prints a summary, and optionally outputs JSON.

#### Options

| Flag | Description |
|---|---|
| `--json` | Output telemetry and datalogger history in JSON format to stdout. |

---

### mppt_live - live polling and settings management

`mppt_live` is the primary tool for a live RS485 connection, each poll cycle issues both a Space command (real-time telemetry) and an EEPROM command (config + datalogger), Settings are read from the EEPROM response which mean no separate command is needed as that's unsupported via RS485.

**Single poll:**
```bash
./build/mppt_live /dev/ttyUSB0
```

**Continuous polling every 30 s with MQTT:**
```bash
./build/mppt_live /dev/ttyUSB0 --loop --broker 192.168.1.10 --zone site1
```

**Read current settings:**
```bash
./build/mppt_live /dev/ttyUSB0 --settings
```

**Write settings** (reads first, patches, writes, re-polls):
```bash
./build/mppt_live /dev/ttyUSB0 --settings --set night_mode=1 --set capacity_ah=100
```

**Poll continuously and write settings in the same run:**
```bash
./build/mppt_live /dev/ttyUSB0 --loop --settings --set evening_min=120 --broker localhost
```

#### Options

| Flag | Default | Description |
|---|---|---|
| `--loop` | off | Poll continuously every 30 s |
| `--settings` | off | Read (and optionally write) device settings |
| `--set key=val` | - | Patch a setting; requires `--settings` |
| `--zone <z>` | `default` | MQTT topic zone segment |
| `--broker <host>` | `localhost` | MQTT broker hostname |
| `--mqtt-port <n>` | `1883` | MQTT broker port |

#### Setting keys

| Key | Range | Notes |
|---|---|---|
| `battery_type` | 0-2 | V2: 0=AGM 1=Liquid 2=LFP; V3: LFP variants |
| `capacity_ah` | 1-500 | |
| `lvd_mv` | 5000-15000 | Low-voltage disconnect threshold (mV) |
| `night_thresh_mv` | 4000-14000 | Night detection threshold (mV) |
| `night_mode` | 0-3 | 0=Off 1=D2D 2=DD 3=MN |
| `evening_min` | 0-600 | |
| `morning_min` | 0-600 | |
| `dim_mode` | 0-3 | |
| `dim_evening_min` | 0-600 | |
| `dim_morning_min` | 0-600 | |
| `dimming_pct` | 0-100 | |
| `base_dimming_pct` | 0-100 | |
| `dali` | 0\|1 | |
| `alc` | 0\|1 | |

---

### mppt_mqtt - publish a log file to MQTT

```bash
./build/mppt_mqtt data_log.txt [--zone <z>] [--broker <h>] [--mqtt-port <n>]
```

Parses a log file and publishes all found telemetry and datalogger records to the specified MQTT broker.

#### Options

| Flag | Default | Description |
|---|---|---|
| `--zone <z>` | `default` | MQTT topic zone segment |
| `--broker <host>` | `localhost` | MQTT broker hostname |
| `--mqtt-port <n>` | `1883` | MQTT broker port |

---

## MQTT topics

| Topic | Description |
|---|---|
| `mppt/{zone}/{gateway}/{serial}/online` | `1` while connected; LWT publishes `0` (retained) |
| `mppt/{zone}/{gateway}/{serial}/info` | Static device identity (HW version, serial), QoS 1, retained |
| `mppt/{zone}/{gateway}/{serial}/state` | Real-time telemetry, QoS 0 |
| `mppt/{zone}/{gateway}/{serial}/datalog` | EEPROM datalogger, QoS 1, retained |
| `mppt/{zone}/{gateway}/{serial}/settings` | Current settings snapshot, QoS 1, retained |
| `mppt/{zone}/{gateway}/{serial}/cmd` | Remote settings command (JSON array of raw commands) |
| `mppt/{zone}/{gateway}/{serial}/ack` | Remote command acknowledgment |
|`mppt/{zone}/{gateway}/{serial}/datalog/summary` | EEPROM history summary | `solar_history` bucket |
| `mppt/{zone}/{gateway}/{serial}/datalog/daily`   | Daily log entries      | `solar_history` bucket |
| `mppt/{zone}/{gateway}/{serial}/datalog/monthly` | Monthly log entries    | `solar_history` bucket |

The `settings` and `info` topics are published once on startup (or when using `--settings`). After a write, `state` and `settings` are re-published to reflect the new configuration.

## Protocol

| Command | Description |
|---|---|
| Space (` `) | Real-time telemetry |
| EEPROM (`!`) | Datalogger history + hardware config + settings |
| `&H` |  Write single EEPROM byte |
| `&M` |  Write EEPROM word (little-endian) |
