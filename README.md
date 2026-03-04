# CNC Monitoring via Industrial IoT

## Project Description
This project implements the firmware architecture for monitoring a CNC machine using Industrial IoT concepts. The system uses an **Arduino Opta** as the master device to collect physical machine data and energy metrics from a **Finder 7M energy meter**, transmitting the information to the cloud through the **Wegnology MQTT Broker**.

The firmware is based on **Mbed OS**, ensuring real-time multitasking operation using an **RTOS architecture**.

---

## Hardware and I/O Architecture

The physical signal mapping on the **Arduino Opta** is structured as follows:

* **A0:** Machine ON (Digital)
* **A1:** Spindle Speed (Analog input 0–10V)
* **A2:** Door Closed (Digital)
* **A3:** Door Open (Digital)
* **A4:** Part Clamped / Clamp (Digital)
* **RS485:** Modbus RTU communication with the Finder 7M energy meter at **38400 baud**

---

## Physical Assembly

Below is the representation of the system's physical assembly, detailing the integration between the CNC machine panel, the Arduino Opta, and the Finder 7M energy meter inside the electrical cabinet.

![assembly](https://github.com/user-attachments/assets/bd2ca228-f3fa-41ba-86a2-1b2bc3e4a9c3)

---

## Software Architecture (RTOS)

The system is divided into four main threads to guarantee real-time multitasking execution:

* **machineThread**  
  Responsible for fast sampling (**50 ms cycles**) and reading the CNC machine sensors.

* **modbusThread**  
  Performs polling through **RS485 serial communication** to retrieve data from the energy meter (voltage, current, power, and frequency).

* **mqttThread**  
  Manages **WiFi connectivity**, **time synchronization (NTP Sync via UDP)**, and telemetry transmission to the cloud.

* **ledManagerThread**  
  Controls visual feedback and local status diagnostics.

---

## Data Structures and Concurrency

Global state management follows the **Single Source of Truth** concept, organizing system data into structured objects:

* **machine_data_t**  
  Stores spindle speed and digital machine states.

* **energy_data_t**  
  Stores electrical quantities such as voltage, current, active power, and power factor.

* **kpi_data_t**  
  Stores operation time, part count, and machine utilization metrics.

To ensure data integrity and prevent race conditions between reading and writing threads, the system implements **Mutexes** (`machineMutex` and `energyMutex`) when accessing shared memory.

---

## KPI Processing and Edge Intelligence

The firmware processes efficiency metrics (**OEE-related indicators**) directly at the edge:

* **Part Counting**  
  Incremented when a **rising edge** is detected on the clamp signal while the spindle is active.

* **Production Rate (Parts/Hour)**  
  Calculated using an **exponential moving average filter** for stability (**Alpha = 0.2**).

* **Utilization Rate (%)**  
  Computed as the ratio between **cutting time** and **machine-on time**.

---

## Telemetry and Data Publishing (MQTT)

Data publishing to the **Wegnology MQTT broker** follows an **event-based approach** to optimize bandwidth usage.

* **Immediate Publishing**  
  Triggered when a state change is detected in the **door, machine status, clamp**, or when spindle speed changes by more than **50 RPM**.

* **Keep-alive (Heartbeat)**  
  Automatic publication every **60 seconds** if no physical state changes occur.

* Payloads are built in **JSON format**, allocated in a **1024-byte buffer**, and include an **ISO-8601 timestamp** synchronized via **NTP**.

* Connectivity includes a **resilience loop** with automatic **MQTT reconnection attempts** (**5000 ms backoff**) to ensure **continuous 24/7 operation**.

---

## Monitoring Dashboards

Below are examples of the processed edge data visualized in the cloud platform.

### Production and OEE Dashboard

Panel displaying the main operational indicators, including **part count**, **machine utilization rate**, and **real-time production rate**.

<img width="1920" height="1080" alt="screenshot" src="https://github.com/user-attachments/assets/7022a33d-7407-4e7e-ace1-81decd611fe7" />

---

### Energy Dashboard

Panel dedicated to electrical monitoring of the equipment, recording **power consumption**, **power factor**, **voltage**, and **current** through **Modbus RTU integration**.

<img width="1920" height="1080" alt="screenshot" src="https://github.com/user-attachments/assets/16fef7fa-5992-45d8-a39d-82e004d0d672" />

---

## Visual Diagnostics

LED indicators on the board simplify field diagnostics of the system status:

* **System LED (D0)**  
  Pulsing signal indicates the operating system is running.

* **WiFi LED (D1)**  
  Slow blinking while connecting and **steady ON** when connected to the network.

* **Modbus LED (D3)**  
  Continuous pulse indicates successful readings, while **fast blinking indicates a read error** with the Finder energy meter.
