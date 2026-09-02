# CNC Monitoring via Industrial IoT

## Project Description

This project implements firmware for monitoring a CNC machine with an **Arduino Opta** using Industrial IoT concepts. The Opta reads machine status signals, collects energy metrics from a **Finder 7M energy meter** over **Modbus RTU/RS485**, calculates production KPIs at the edge, and publishes telemetry to an MQTT broker.

The firmware is built with **PlatformIO**, using the **Arduino framework** on top of the Opta's Mbed-based runtime. The application is organized as an RTOS firmware with independent tasks for machine acquisition, energy acquisition, telemetry, and diagnostics.

---

## Hardware and I/O Architecture

The physical signal mapping on the **Arduino Opta** is configured in `include/Config.h` and can be overridden through PlatformIO build flags.

Default input mapping:

- **A0:** Machine ON
- **A1:** Spindle running
- **A2:** Door closed
- **A3:** Door opened
- **A4:** Part clamped
- **A5:** Machine ready
- **RS485:** Modbus RTU communication with the Finder 7M energy meter at **38400 baud**

Default diagnostic LED mapping:

- **D0:** System status
- **D1:** Wi-Fi status
- **D2:** MQTT status
- **D3:** Modbus/Finder 7M status

---

## Physical Assembly

Below is the representation of the system's physical assembly, detailing the integration between the CNC machine panel, the Arduino Opta, and the Finder 7M energy meter inside the electrical cabinet.

![assembly](https://github.com/user-attachments/assets/bd2ca228-f3fa-41ba-86a2-1b2bc3e4a9c3)

---

## Software Architecture

The firmware is split into domain, application, and platform layers:

- `src/domain` and `include/domain` contain hardware-independent rules such as KPI processing, telemetry serialization, and publishing policy.
- `src/application` and `include/application` compose the firmware and coordinate shared state.
- `src/platform` and `include/platform` contain hardware adapters for Opta inputs, Finder 7M readings, Wi-Fi/MQTT, diagnostics, logging, and RTOS wrappers.

The application starts four main RTOS threads:

- **machineThread**  
  Samples CNC machine signals every **50 ms**, updates machine state, and recalculates KPIs.

- **energyThread**  
  Polls the Finder 7M energy meter every **5000 ms** through RS485/Modbus RTU.

- **telemetryThread**  
  Manages Wi-Fi, NTP synchronization, MQTT reconnection, JSON serialization, and telemetry publishing.

- **diagnosticThread**  
  Updates local LED diagnostics for system, Wi-Fi, MQTT, and Modbus status.

---

## Data Model

The firmware maintains a coherent system snapshot composed of:

- **MachineData**  
  Machine ON, machine ready, door closed/opened, part clamped, and spindle running states.

- **EnergyData**  
  Voltage, current, active power, reactive power, apparent power, power factor, frequency, and accumulated active energy.

- **KpiData**  
  Machine-on time, operation time, utilization rate, part count, cycle time, and production cadence.

Shared state is centralized through `StateStore`, keeping acquisition, processing, and publishing logic separated.

---

## KPI Processing and Edge Intelligence

The firmware processes production indicators directly at the edge:

- **Part counting**  
  Incremented when a rising edge is detected on the part-clamped signal.

- **Operation time**  
  Accumulated when the spindle is running and the part is clamped.

- **Machine-on time**  
  Accumulated while the machine-on signal is active.

- **Utilization rate (%)**  
  Calculated as operation time divided by machine-on time.

- **Cycle time and cadence**  
  Calculated from the interval between detected clamp events. Cadence uses an exponential moving average filter with **alpha = 0.2**.

---

## Telemetry and MQTT Publishing

Telemetry is published as JSON to the configured MQTT broker and topic.

Publishing is event-based:

- A message is published immediately after relevant machine state changes.
- A heartbeat message is published every **60 seconds** if no state changes occur.
- Wi-Fi and MQTT reconnection attempts use a **5000 ms** interval.
- Timestamps are generated in ISO-8601 format after NTP synchronization.
- Payloads are serialized into a local **600-byte** buffer.

Example payload:

```json
{
  "time": "2026-08-01T12:00:00Z",
  "data": {
    "voltageV": 220.00,
    "currentA": 4.20,
    "activePowerW": 850.00,
    "reactivePowerVAr": 120.00,
    "apparentPowerVA": 910.00,
    "powerFactor": 0.930,
    "frequencyHz": 60.00,
    "activeEnergyKWh": 15.420,
    "machineOn": 1,
    "machineReady": 1,
    "doorClosed": 1,
    "doorOpened": 0,
    "partClamped": 1,
    "spindleRunning": 1,
    "machineOnTimeS": 3600,
    "operationTimeS": 2700,
    "utilizationRate": 75.0,
    "partCount": 120,
    "cycleTimeS": 30.000,
    "cadence": 120.0
  }
}
```

---

## PlatformIO Setup

Required PlatformIO environment:

```ini
[env:opta]
platform = ststm32
board = opta
framework = arduino
```

Project dependencies:

```ini
lib_deps =
	dndg/Finder 7M for Finder Opta@^1.1.5
	arduino-libraries/ArduinoMqttClient@^0.1.8
	arduino-libraries/NTPClient@^3.2.1
```

Typical commands:

```bash
pio run
pio run --target upload
pio device monitor
```

---

## Monitoring Dashboards

Below are examples of the processed edge data visualized in the cloud platform.

### Production and OEE Dashboard

Panel displaying the main operational indicators, including part count, machine utilization rate, cycle time, and real-time production cadence.

<img width="1920" height="1080" alt="Production and OEE dashboard" src="https://github.com/user-attachments/assets/7022a33d-7407-4e7e-ace1-81decd611fe7" />

---

### Energy Dashboard

Panel dedicated to electrical monitoring of the equipment, recording power consumption, power factor, voltage, current, frequency, and accumulated energy through Modbus RTU integration.

<img width="1920" height="1080" alt="Energy dashboard" src="https://github.com/user-attachments/assets/16fef7fa-5992-45d8-a39d-82e004d0d672" />

---

## Visual Diagnostics

LED indicators on the Opta simplify field diagnostics:

- **System LED (D0)**  
  Slow blinking indicates that the firmware is running.

- **Wi-Fi LED (D1)**  
  Slow blinking while connecting and steady ON when connected.

- **MQTT LED (D2)**  
  Slow blinking while disconnected and steady ON when connected to the broker.

- **Modbus LED (D3)**  
  Pulse indicates successful Finder 7M readings. Fast blinking indicates initialization or read errors.

---

## Local Secrets Workflow

This repository includes `secrets.example.ini` as a safe template. The real `secrets.ini` file is ignored by Git and must stay local because it contains Wi-Fi and MQTT credentials.

Before building or flashing the firmware, copy the example file:

```bash
cp secrets.example.ini secrets.ini
```

On Windows PowerShell:

```powershell
Copy-Item secrets.example.ini secrets.ini
```

Then edit `secrets.ini` with the real values for:

- Wi-Fi SSID and password
- MQTT broker address and port
- MQTT client ID
- MQTT username and password
- MQTT topic

---

## License

This project is licensed under the MIT License. See `LICENSE` for details.
