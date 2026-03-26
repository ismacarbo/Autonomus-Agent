# Robot Architecture Overview

This document provides a high-level overview of the robot architecture, including
hardware layers, software layers, communication protocols, and the robot state machine.

It complements the following specifications:

- **RSP-v1** — Robot Serial Protocol
- **RMSM-v1** — Robot Mode and State Machine Model

The goal is to provide a clear conceptual map of the system.

---

# System Architecture

The robot architecture is organized in layered components.

```mermaid
graph TD

User[Operator / Developer]

Host[High Level Controller<br>Raspberry Pi]
Planner[Planning / Navigation]
Perception[Perception & Mapping]
Protocol[RSP Protocol Layer]

MCU[Low Level Controller<br>Arduino]
MotorCtrl[Motor Control]
Sensors[MPU6050 / AO Encoders]
Lidar[RPLidar A1]

Hardware[Motors & Hardware]

User --> Host
Host --> Planner
Host --> Perception
Host --> Protocol
Lidar --> Host

Protocol --> MCU

MCU --> MotorCtrl
MCU --> Sensors

MotorCtrl --> Hardware
```
---

# Software Layer Stack

The robot software can be divided into layers.

```mermaid
graph TD

Application[Autonomy / Navigation]

Mapping[SLAM / Mapping]
Control[Motion Control]

Protocol[RSP Protocol]

DriverMCU[Motor Driver]
DriverSensors[Sensor Drivers]

HardwareLayer[Hardware]

Application --> Mapping
Application --> Control

Mapping --> Protocol
Control --> Protocol

Protocol --> DriverMCU
Protocol --> DriverSensors

DriverMCU --> HardwareLayer
DriverSensors --> HardwareLayer
```
---

# Robot State Machine

The robot behavior is governed by the **RMSM-v1 finite state machine**.

```mermaid
stateDiagram-v2

[*] --> BOOT

BOOT --> IDLE : setup_complete

IDLE --> MANUAL : MODE_CMD(manual)
IDLE --> AUTONOMOUS : MODE_CMD(autonomous)
IDLE --> CALIBRATING : GYRO_ZERO_CMD / MODE_CMD(calibration)

MANUAL --> IDLE : MODE_CMD(idle)
AUTONOMOUS --> IDLE : MODE_CMD(idle)
CALIBRATING --> IDLE : calibration_done

MANUAL --> CALIBRATING : GYRO_ZERO_CMD
AUTONOMOUS --> CALIBRATING : GYRO_ZERO_CMD

IDLE --> EMERGENCY_STOP : MODE_CMD(emergency_stop) / fault_latch
MANUAL --> EMERGENCY_STOP : MODE_CMD(emergency_stop) / fault_latch
AUTONOMOUS --> EMERGENCY_STOP : MODE_CMD(emergency_stop) / fault_latch
CALIBRATING --> EMERGENCY_STOP : fault_latch

EMERGENCY_STOP --> IDLE : MODE_CMD(idle) / recovery
```
---

# Data Flow Overview

Sensors and commands propagate through the system.

```mermaid
graph LR

Encoders[Wheel Encoders]
Imu[MPU6050]
Lidar[RPLidar A1]
MCU[Arduino Controller]
Serial[RSP Serial Bus]
Host[High Level Controller]
Algorithms[Navigation / SLAM]
Commands[Motor Commands]
Motors[Motors]

Encoders --> MCU
Imu --> MCU
MCU --> Serial
Serial --> Host
Lidar --> Host
Host --> Algorithms
Algorithms --> Commands
Commands --> Serial
Serial --> MCU
MCU --> Motors
```
---

# Communication Channels

| Channel | Description |
|-------|-------------|
| Serial (RSP) | Primary communication between Raspberry Pi and Arduino |
| Internal MCU | Direct sensor and actuator control |
| Optional Network | Future WiFi / ROS integration |

---

# Hardware Architecture

```mermaid
graph TD

Pi[Raspberry Pi]

Arduino[Arduino Controller]

IMU[IMU Sensor]
Encoders[4x AO Wheel Encoders]
Lidar[RPLidar A1]

Driver[TB6612FNG Motor Driver]

Motors[DC Motors]

Pi --> Arduino
Pi --> Lidar

Arduino --> IMU
Arduino --> Encoders

Arduino --> Driver
Driver --> Motors
```
---

# Design Philosophy

The architecture follows several design principles.

### Separation of Responsibilities

Low-level controller:
- deterministic control
- real-time sensor access
- motor safety

High-level controller:
- perception
- planning
- mapping
- autonomy

---

### Protocol Isolation

The RSP protocol acts as a **stable interface** between:

- low-level firmware
- high-level software

This allows either side to evolve independently.

---

### Safety First

Safety decisions must always override motion commands.

The FSM ensures:

- safe stopping
- deterministic transitions
- clear recovery behavior

On the current hardware profile:

- low-level safety is mainly command timeout, stop requests, and emergency-stop latching
- obstacle detection is performed primarily on the Raspberry Pi from LiDAR data

---

# Future Extensions

Possible extensions include:

- CAN bus communication
- ROS2 integration
- multi-sensor fusion
- depth camera perception
- distributed robot architecture

---

# Summary

The robot system consists of:

- layered software architecture
- low-level real-time controller
- high-level autonomy controller
- binary serial communication protocol
- finite state machine governing behavior

Together these components provide a robust foundation for building increasingly complex autonomous capabilities.
