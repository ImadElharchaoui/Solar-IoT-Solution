# MPPT Solar Monitor - Cloud MING Stack

Containerised monitoring solution for MPPT solar charge controllers, based on **Mosquitto - InfluxDB - Node-RED - Grafana**.

## Services

| Service | Port | Purpose |
|---|---|---|
| Mosquitto | 1883 | MQTT broker - ingests data from `mppt_live` |
| InfluxDB | 8086 | Time-series storage |
| Node-RED | 1880 | Routes MQTT messages into InfluxDB |
| Grafana | 3000 | Dashboard visualisation |
| MQTT Explorer | 4000 | Browser-based MQTT debug tool |

## Quick Start

**1. Create a `.env` file** in this directory (an example file is already included):

```bash
INFLUX_ADMIN_PASSWORD=your_strong_password
INFLUX_TOKEN=your_token_for_node_red
GRAFANA_ADMIN_PASSWORD=your_grafana_password
SMTP_HOST=smtp.example.com
SMTP_PORT=587
SMTP_USER=user@example.com
SMTP_PASS=password
ALERT_EMAIL=alerts@example.com
NODE_RED_CREDENTIAL_SECRET=your_secret_key
```

**2. Start all services:**

```bash
docker-compose up -d
```

**3. Access the UIs:**

- Node-RED: http://localhost:1880
- Node-RED Dashboard: http://localhost:1880/dashboard
- InfluxDB: http://localhost:8086
- Grafana: http://localhost:3000 (user: `admin`)
- MQTT Explorer: http://localhost:4000

## Data Flow

`mppt_live` publishes to MQTT -> Node-RED writes to InfluxDB -> Grafana queries InfluxDB.

### MQTT Topic Structure

Node-RED processes the following topics:

| MQTT topic | Purpose | InfluxDB Mapping |
|---|---|---|
| `mppt/{zone}/{gateway}/{serial}/state` | Real-time telemetry | `solar` bucket (telemetry) |
| `mppt/{zone}/{gateway}/{serial}/datalog` | EEPROM history | `solar_history` bucket |
| `mppt/{zone}/{gateway}/{serial}/settings` | Device config | UI status / Dashboard |
| `mppt/{zone}/{gateway}/{serial}/info` | Device identity | Fleet management |
| `mppt/{zone}/{gateway}/{serial}/online` | Availability | LWT / Status indicators |
| `mppt/{zone}/{gateway}/{serial}/cmd` | Control actions | High-level commands (e.g. `SET_SETTINGS`) |
| `mppt/{zone}/{gateway}/{serial}/ack` | Command feedback | UI notification / Validation |

Topic segments map to InfluxDB tags (`zone`, `gateway_id`, `serial`) for granular filtering in Grafana.

## Provisioning

- **Grafana** dashboards are pre-provisioned from `grafana/provisioning/dashboards/`.
- **InfluxDB** buckets and access policies are initialised by `influxdb/init/init.sh` on first start.
- **Node-RED** flow data is persisted in `nodered/data/`.
