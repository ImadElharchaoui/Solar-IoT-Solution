# Solar-IoT-Solution

A comprehensive IoT solution for monitoring and managing MPPT (Maximum Power Point Tracking) solar charge controllers. This project provides highC++ tools for data parsing and a containerized monitoring stack (MING: Mosquitto, InfluxDB, Node-RED, Grafana).

## Architecture

The system follows a multi-stage data pipeline:

1.  **Edge Devices:** MPPT controllers connected via RS485 to gateways (e.g., Raspberry Pi or Linux industrial PCs).
2.  **Data Ingestion:** C++ based tools (`mppt_live`) poll the controllers, parse raw telemetry and historical data, and publish it to an MQTT broker.
3.  **Persistence & Logic:** Node-RED subscribes to MQTT topics, processes payloads, and stores data in InfluxDB 2.x. It also handles alerting and high-level control commands.
4.  **Visualization:** Grafana provides real-time and historical dashboards for monitoring solar performance, battery health, and load status.

## Project Structure

-   [`/parser`](./parser): C++ tools for decoding RS485 protocol, managing settings, and publishing to MQTT.
-   [`/mppt-stack`](./mppt-stack): Docker Compose configuration for the MING stack.
-   [`/proto`](./proto): Protobuf definitions for device settings and state (ensures consistent schema across C++, Python, and Node-RED).
-   [`/rpi`](./rpi): Python scripts and utilities for Raspberry Pi gateways (utilizes Protobuf for command handling).

## Prerequisites

-   **Docker & Docker Compose:** For running the MING monitoring stack.
-   **C++17 Toolchain:** `g++`, `make`.
-   **Libraries:** `paho-mqtt-cpp`, `nlohmann-json`.
-   **Protobuf:** `libprotobuf-dev`, `protobuf-compiler` (for C++), and `python3-protobuf` (for Python).

## Getting Started

### 1. Build the Parser
Requires a C++17 compiler and `paho-mqtt-cpp`.
```bash
cd parser
make release
```

### 2. Launch the Monitoring Stack
1.  Create a `.env` file in `mppt-stack/` (see `mppt-stack/README.md` for template).
2.  Start the services:
    ```bash
    cd mppt-stack
    docker-compose up -d
    ```

### 3. Start Monitoring
Connect your MPPT controller via RS485 and start the live poller:
```bash
./parser/build/mppt_live /dev/ttyUSB0 --loop --broker localhost
```

## Documentation

-   [MPPT Parser & CLI Tools](./parser/README.md)
-   [IoT Stack & Dashboard Setup](./mppt-stack/README.md)

