# Robot Serial Protocol v1 (RSP-v1)

## Purpose

**Robot Serial Protocol v1 (RSP-v1)** is a binary serial communication protocol for a mobile robot architecture composed of:

- a **low-level controller** responsible for actuation and direct sensor acquisition
- a **high-level host** responsible for perception, planning, supervision, and internal message routing

The protocol is designed to be:

- robust
- compact
- deterministic
- extensible
- independent from the internal host middleware

Its purpose is to define a clean transport and framing layer between the embedded controller and the host computer.

---

## Scope

This note defines:

- protocol philosophy
- communication model
- frame structure
- message families
- validation model
- acknowledgment model
- error model
- timing assumptions
- versioning policy

This note intentionally does **not** define implementation details or code-level parsing logic.

---

## Architectural Context

### System Roles

#### Low-Level Controller
The low-level controller is the embedded device directly connected to motors and near-hardware sensors. Its typical responsibilities are:

- motor actuation
- low-level safety behavior
- IMU acquisition
- proximity sensor acquisition
- encoder acquisition
- execution of host commands
- continuous state reporting

#### High-Level Host
The high-level host is the main computational node. Its typical responsibilities are:

- receiving and validating serial frames
- decoding payloads
- routing decoded data into an internal bus
- integrating LiDAR, vision, or other high-bandwidth sensors
- running perception, localization, mapping, and control logic
- issuing motion and mode commands to the low-level controller

---

## Core Design Principles

### 1. Transport and Middleware Must Remain Separate

The serial protocol is a **transport protocol**.
It is not a topic system, not a ROS replacement, and not the full software architecture.

A higher-level message bus may exist on the host, but that is a separate layer.

### 2. Frames Must Be Self-Contained

Each frame must provide enough structure to allow the receiver to determine:

- where the frame begins
- what kind of message it carries
- how long the payload is
- whether the payload is valid
- whether the frame is complete and intact

### 3. Telemetry and Commands Must Be Clearly Distinguished

A command changes state or requests an action.
Telemetry reports the current state of the system.

This distinction is important for:

- reliability
- logging clarity
- acknowledgment policy
- safety reasoning

### 4. The Protocol Must Scale with the Robot

The first version must remain small enough for constrained hardware, but the design must already support future growth, including:

- wheel encoders
- additional sensors
- mode changes
- richer status reporting
- host-side bus integration
- future transport abstraction changes

### 5. Payload Semantics Must Be Explicit

Every field in every payload must have:

- a defined type
- a defined unit
- a defined meaning
- a defined valid range or interpretation

Ambiguity at the protocol level leads to long-term architectural confusion.

---

## Communication Model

### Directionality

The protocol supports bidirectional communication.

#### Host to Low-Level Controller
Typical messages include:

- motion commands
- stop requests
- mode changes
- calibration commands
- heartbeat or liveness frames
- configuration updates

#### Low-Level Controller to Host
Typical messages include:

- IMU telemetry
- safety telemetry
- encoder telemetry
- motor state
- heartbeat state
- error notifications

---

## Transport Assumptions

### Physical Link

RSP-v1 assumes a UART serial link between the two nodes.
The physical transport is considered a byte stream. The protocol itself provides the structure needed to delimit and validate frames.

### Endianness

All multibyte fields use:

- **little-endian encoding**

This applies consistently across the entire protocol.

---

## Frame Structure

Every message is encoded as a binary frame with three conceptual sections:

1. frame preamble
2. frame body
3. frame integrity field

### Canonical Frame Layout

| Field      |        Size | Type       | Meaning                      |
| ---------- | ----------: | ---------- | ---------------------------- |
| `SOF1`     |      1 byte | `uint8`    | First start-of-frame marker  |
| `SOF2`     |      1 byte | `uint8`    | Second start-of-frame marker |
| `VERSION`  |      1 byte | `uint8`    | Protocol version             |
| `MSG_TYPE` |      1 byte | `uint8`    | Message identifier           |
| `FLAGS`    |      1 byte | `uint8`    | Frame metadata flags         |
| `SEQ`      |      1 byte | `uint8`    | Sequence number              |
| `LEN`      |     2 bytes | `uint16`   | Payload length               |
| `PAYLOAD`  | `LEN` bytes | byte array | Message content              |
| `CRC16`    |     2 bytes | `uint16`   | Frame integrity check        |

### Start-of-Frame Markers

The protocol uses two dedicated start bytes:

- `SOF1 = 0xAA`
- `SOF2 = 0x55`

These markers help the receiver identify the beginning of a frame and recover from corrupted or misaligned input streams.

### Version Field

`VERSION` identifies the protocol version.

For this specification:

- `VERSION = 0x01`

### Message Type Field

`MSG_TYPE` identifies the semantic category of the payload.
Each message type has exactly one associated payload definition.

### Flags Field

`FLAGS` carries frame-level metadata.

|  Bit | Name        | Meaning                         |
| ---: | ----------- | ------------------------------- |
|    0 | `ACK_REQ`   | Sender requests acknowledgment  |
|    1 | `ACK_FRAME` | This frame is an acknowledgment |
|    2 | `ERR_FRAME` | This frame reports an error     |
|    3 | `RESERVED`  | Reserved                        |
|    4 | `RESERVED`  | Reserved                        |
|    5 | `RESERVED`  | Reserved                        |
|    6 | `RESERVED`  | Reserved                        |
|    7 | `RESERVED`  | Reserved                        |

Reserved bits must be transmitted as zero in v1.

### Sequence Number

`SEQ` is a rolling sequence number assigned by the sender.
It is used for:

- ordering
- diagnostics
- associating acknowledgments with prior frames
- detecting loss or duplication if needed

### Length Field

`LEN` defines the payload length in bytes.
Although the type permits large theoretical values, protocol-constrained devices should define a much smaller practical maximum.

### Payload Field

`PAYLOAD` contains the message-specific data.
Its exact structure depends entirely on `MSG_TYPE`.

### CRC Field

`CRC16` protects the frame from corruption.

RSP-v1 uses:

- **CRC-16/CCITT-FALSE**

#### CRC Parameters

| Parameter        | Value    |
| ---------------- | -------- |
| Polynomial       | `0x1021` |
| Initial value    | `0xFFFF` |
| Reflected input  | `false`  |
| Reflected output | `false`  |
| Final XOR        | `0x0000` |

#### CRC Coverage

CRC is computed over:

- `VERSION`
- `MSG_TYPE`
- `FLAGS`
- `SEQ`
- `LEN`
- `PAYLOAD`

The start-of-frame markers are not part of the CRC domain.

---

## Conceptual Message Families

### 1. Control Messages

These frames instruct the low-level controller to perform an action or enter a state.

Examples:

- motor command
- stop request
- mode switch
- gyro recalibration
- configuration write

### 2. Telemetry Messages

These frames communicate measured or estimated system state.

Examples:

- IMU telemetry
- safety telemetry
- encoder telemetry
- motor state
- low-level heartbeat state

### 3. Service Messages

These frames support protocol-level coordination rather than robot behavior directly.

Examples:

- ping
- acknowledgment
- error report
- heartbeat command

---

## Message Catalogue

### Service / Protocol-Level Messages

| Name    |     ID | Direction          |
| ------- | -----: | ------------------ |
| `PING`  | `0x01` | Host -> Controller |
| `ACK`   | `0x02` | Bidirectional      |
| `ERROR` | `0x03` | Bidirectional      |

### Control Messages

| Name            |     ID | Direction          |
| --------------- | -----: | ------------------ |
| `MOTOR_CMD`     | `0x10` | Host -> Controller |
| `STOP_CMD`      | `0x11` | Host -> Controller |
| `MODE_CMD`      | `0x12` | Host -> Controller |
| `GYRO_ZERO_CMD` | `0x13` | Host -> Controller |
| `CONFIG_SET`    | `0x14` | Host -> Controller |
| `HEARTBEAT_CMD` | `0x15` | Host -> Controller |

### Telemetry Messages

| Name                |     ID | Direction          |
| ------------------- | -----: | ------------------ |
| `IMU_TELEMETRY`     | `0x20` | Controller -> Host |
| `SAFETY_TELEMETRY`  | `0x21` | Controller -> Host |
| `ENCODER_TELEMETRY` | `0x22` | Controller -> Host |
| `MOTOR_STATE`       | `0x23` | Controller -> Host |
| `HEARTBEAT_STATE`   | `0x24` | Controller -> Host |

---

## Message Semantics

### `PING` (`0x01`)
A protocol-level liveness probe used to verify transport-level reachability.

### `ACK` (`0x02`)
A confirmation that a prior frame was successfully received and accepted.
An ACK should refer to a specific prior frame.

### `ERROR` (`0x03`)
A protocol or command-handling error report.
It is used when a frame is parsed correctly but cannot be accepted or executed.

### `MOTOR_CMD` (`0x10`)
A differential drive command issued by the host.
It expresses motion intent toward the low-level controller.

### `STOP_CMD` (`0x11`)
An explicit stop request.
It has stronger semantic meaning than merely sending a zero motor command.

### `MODE_CMD` (`0x12`)
A request to change the controller's operation mode.
Examples include idle, manual, autonomous, calibration, and emergency stop latch.

### `GYRO_ZERO_CMD` (`0x13`)
A request to recalibrate gyroscope bias and reset integrated heading state.
It is a state-changing command and should be treated as critical.

### `CONFIG_SET` (`0x14`)
A generic configuration message that supports parameter-level adjustment and controlled extensibility.

### `HEARTBEAT_CMD` (`0x15`)
A host-originated keepalive frame used to support watchdog and link-health policies.

### `IMU_TELEMETRY` (`0x20`)
An inertial telemetry message carrying the local inertial state used by the host for estimation and control.

### `SAFETY_TELEMETRY` (`0x21`)
A telemetry message carrying low-level proximity and safety-related information.

### `ENCODER_TELEMETRY` (`0x22`)
A telemetry message reporting wheel encoder state. The protocol conceptually favors cumulative counts over deltas.

### `MOTOR_STATE` (`0x23`)
A telemetry message describing the low-level motor actuation state. It reports what is actually happening, not only commanded intent.

### `HEARTBEAT_STATE` (`0x24`)
A general low-level system state message summarizing health and readiness.

---

## Payload Design Philosophy

All payloads in RSP-v1 should follow these rules.

### 1. Use Explicit Units
Every numeric field must correspond to a defined unit.

### 2. Prefer Integer-Based Representations
For v1, scaled integers are preferred over floating-point representations at the protocol level.

### 3. Use Status Flags for Orthogonal State
Flags are appropriate when several independent boolean conditions must be reported together.

### 4. Keep Payload Semantics Stable
A field should not change meaning depending on context.

---

## Acknowledgment Policy

### Messages That Typically Require ACK
ACK is appropriate when a message:

- changes controller state
- requests a nontrivial action
- is safety-critical
- should not be silently ignored

Typical examples:

- `STOP_CMD`
- `MODE_CMD`
- `GYRO_ZERO_CMD`
- `CONFIG_SET`
- diagnostic `PING`

### Messages That Typically Do Not Require ACK
Continuous streams generally do not require ACK because the next sample supersedes the previous one.

Typical examples:

- `IMU_TELEMETRY`
- `SAFETY_TELEMETRY`
- `ENCODER_TELEMETRY`
- `MOTOR_STATE`
- `HEARTBEAT_STATE`
- continuous `MOTOR_CMD`

---

## Timing Model

The protocol distinguishes between two types of time.

### Device Time
Time measured by the low-level controller.
This indicates when the data was produced or the low-level state existed.

### Host Time
Time assigned by the host when a valid frame is received and decoded.

Both are important:

- device time supports sensor interpretation and fusion
- host time supports latency diagnostics and end-to-end monitoring

---

## Reliability and Validation Model

A frame should be considered valid only if all of the following are true:

1. the start markers are correct
2. the version is supported
3. the declared payload length is within allowed bounds
4. the full frame is present
5. the CRC matches
6. the message type is known
7. the payload structure is semantically acceptable for that message type

Any failure means the frame must not be accepted as valid protocol data.

---

## Corruption and Resynchronization

Since serial is a byte stream, corruption or byte loss can desynchronize the receiver.
The protocol is designed so the receiver can recover by scanning for the two-byte start-of-frame pattern and then attempting a fresh decode.

---

## Error Model

The protocol distinguishes between two classes of failure.

### 1. Structural Failures
These happen before semantic interpretation.
Examples include invalid start sequence, unsupported version, invalid length, CRC mismatch, or truncated frame.

### 2. Semantic Failures
These happen after a frame is correctly parsed.
Examples include unsupported command, invalid mode, out-of-range values, unavailable subsystems, or calibration busy.

### Standard Error Categories

|   Code | Name                   | Meaning                                            |
| -----: | ---------------------- | -------------------------------------------------- |
| `0x01` | `UNKNOWN_MSG_TYPE`     | Message type not recognized                        |
| `0x02` | `INVALID_LENGTH`       | Payload length inconsistent with type              |
| `0x03` | `CRC_MISMATCH`         | Integrity check failed                             |
| `0x04` | `INVALID_VALUE`        | Field value outside accepted domain                |
| `0x05` | `IMU_NOT_READY`        | IMU unavailable or uninitialized                   |
| `0x06` | `CALIBRATION_BUSY`     | Requested action blocked by calibration            |
| `0x07` | `MOTORS_DISABLED`      | Motion command rejected because motors unavailable |
| `0x08` | `ENCODERS_UNAVAILABLE` | Encoder-related state unavailable                  |
| `0x09` | `SENSOR_TIMEOUT`       | Sensor acquisition timed out                       |
| `0x0A` | `UNSUPPORTED_MODE`     | Requested mode invalid or unsupported              |
| `0x0B` | `INTERNAL_FAULT`       | Internal controller fault                          |
| `0x0C` | `BUSY`                 | Resource or subsystem temporarily busy             |
| `0x0D` | `NOT_IMPLEMENTED`      | Recognized but not implemented in current firmware |

---

## Versioning Policy

### Version 1
Version 1 defines:

- the binary frame structure
- the message families
- the initial set of message identifiers
- the acknowledgment model
- the error model

### Future Versions
A later version may introduce:

- new message types
- new flags
- extended payload semantics
- additional status models
- alternate scaling conventions

---

## Relationship with the Host Message Bus

The serial protocol remains **numeric and compact**, while the host internal bus may remain **semantic and topic-based**.

That means:

- the embedded controller transmits compact binary message types
- the host may map them to internal topics such as `robot/imu/raw` or `robot/motor/state`

This separation keeps the transport lean while preserving architectural clarity on the host side.

---

## Recommended Minimal v1 Scope

For a clean first deployment, the most meaningful subset of the protocol is:

### Commands
- `MOTOR_CMD`
- `STOP_CMD`
- `GYRO_ZERO_CMD`
- `ACK`
- `ERROR`

### Telemetry
- `IMU_TELEMETRY`
- `SAFETY_TELEMETRY`
- `MOTOR_STATE`
- `HEARTBEAT_STATE`

### Planned Extension
- `ENCODER_TELEMETRY` when encoder hardware is integrated

This minimal scope is sufficient to establish the architecture correctly without overloading the first implementation phase.

---

## Summary

RSP-v1 defines a binary serial protocol intended to serve as the communication backbone between a low-level robot controller and a high-level host.

Its essential properties are:

- explicit frame boundaries
- explicit message typing
- explicit payload lengths
- integrity checking through CRC
- clear separation between telemetry, commands, and service messages
- version-aware extensibility
- independence from host-side middleware design

The key architectural takeaway is:

> RSP-v1 is the protocol between embedded control and host supervision.
> It is not the host middleware itself.

---

## Compact Reference

### Frame Header

| Field      | Size |
| ---------- | ---: |
| `SOF1`     |    1 |
| `SOF2`     |    1 |
| `VERSION`  |    1 |
| `MSG_TYPE` |    1 |
| `FLAGS`    |    1 |
| `SEQ`      |    1 |
| `LEN`      |    2 |

### Frame Footer

| Field   | Size |
| ------- | ---: |
| `CRC16` |    2 |

### Start Bytes

| Field  | Value  |
| ------ | ------ |
| `SOF1` | `0xAA` |
| `SOF2` | `0x55` |

### Protocol Version

| Field     | Value  |
| --------- | ------ |
| `VERSION` | `0x01` |

### CRC Algorithm

| Field      | Value                |
| ---------- | -------------------- |
| Algorithm  | `CRC-16/CCITT-FALSE` |
| Polynomial | `0x1021`             |
| Init       | `0xFFFF`             |

### Core Message IDs

|     ID | Name                |
| -----: | ------------------- |
| `0x01` | `PING`              |
| `0x02` | `ACK`               |
| `0x03` | `ERROR`             |
| `0x10` | `MOTOR_CMD`         |
| `0x11` | `STOP_CMD`          |
| `0x12` | `MODE_CMD`          |
| `0x13` | `GYRO_ZERO_CMD`     |
| `0x14` | `CONFIG_SET`        |
| `0x15` | `HEARTBEAT_CMD`     |
| `0x20` | `IMU_TELEMETRY`     |
| `0x21` | `SAFETY_TELEMETRY`  |
| `0x22` | `ENCODER_TELEMETRY` |
| `0x23` | `MOTOR_STATE`       |
| `0x24` | `HEARTBEAT_STATE`   |
