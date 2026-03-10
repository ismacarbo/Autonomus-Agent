# RSP-v1 Payload Definitions and Units

## Purpose

This note complements the main **Robot Serial Protocol v1 (RSP-v1)** specification.
Its purpose is to define the payloads of all protocol messages at the specification level, without discussing implementation.

For each message, this note describes:

- semantic role
- payload fields
- field types
- units
- interpretation rules
- flag meanings
- acknowledgment expectations where relevant

---

## General Payload Rules

### Field Encoding
All multibyte fields are encoded in:

- **little-endian**

### Numeric Representation
RSP-v1 prefers:

- integer fields
- scaled integer physical quantities
- explicit units for every value

### Reserved Fields
Any field marked as reserved must:

- be transmitted as zero in v1
- be ignored by the receiver unless otherwise specified in a future version

### Empty Payloads
Some messages intentionally have no payload. In those cases:

- `LEN = 0`

---

## Service / Protocol-Level Messages

---

## `PING` (`0x01`)

### Role
A transport-level liveness probe.

### Direction
- Host -> Controller

### Payload
This message has no payload.

| Field | Size | Type | Description |
| ----- | ---: | ---- | ----------- |
| -     |    0 | -    | No payload  |

### Notes
`PING` is intended for diagnostics and reachability verification.
It is conceptually separate from control logic.

---

## `ACK` (`0x02`)

### Role
Acknowledge receipt and acceptance of a prior frame.

### Direction
- Bidirectional

### Payload

| Field        | Size | Type    | Unit | Description                           |
| ------------ | ---: | ------- | ---- | ------------------------------------- |
| `ACKED_SEQ`  |    1 | `uint8` | -    | Sequence number of acknowledged frame |
| `ACKED_TYPE` |    1 | `uint8` | -    | Message type being acknowledged       |
| `STATUS`     |    1 | `uint8` | enum | Acknowledgment status                 |
| `DETAIL`     |    1 | `uint8` | -    | Additional detail or substatus        |

### `STATUS` Values

|  Value | Meaning                              |
| -----: | ------------------------------------ |
| `0x00` | Accepted and completed               |
| `0x01` | Accepted, action pending             |
| `0x02` | Accepted, already in requested state |
| `0x03` | Rejected                             |

### Notes
An ACK should refer to exactly one previously transmitted frame.

---

## `ERROR` (`0x03`)

### Role
Report a semantic or operational failure after valid parsing.

### Direction
- Bidirectional

### Payload

| Field          | Size | Type    | Unit | Description                               |
| -------------- | ---: | ------- | ---- | ----------------------------------------- |
| `ERROR_CODE`   |    1 | `uint8` | enum | Primary error category                    |
| `RELATED_TYPE` |    1 | `uint8` | -    | Message type associated with the error    |
| `RELATED_SEQ`  |    1 | `uint8` | -    | Sequence number associated with the error |
| `DETAIL`       |    1 | `uint8` | -    | Additional error detail                   |

### Notes
This message is for meaningful semantic failures, not for arbitrary corrupted bytes that fail framing or CRC checks.

---

## Control Messages

---

## `MOTOR_CMD` (`0x10`)

### Role
Differential drive motor command sent by the host.

### Direction
- Host -> Controller

### Payload

| Field          | Size | Type    | Unit      | Description                         |
| -------------- | ---: | ------- | --------- | ----------------------------------- |
| `PWM_LEFT`     |    2 | `int16` | PWM units | Requested left motor command        |
| `PWM_RIGHT`    |    2 | `int16` | PWM units | Requested right motor command       |
| `CONTROL_MODE` |    1 | `uint8` | enum      | Interpretation mode for the command |
| `RESERVED`     |    1 | `uint8` | -         | Reserved, must be zero              |

### `CONTROL_MODE` Values

|  Value | Meaning                           |
| -----: | --------------------------------- |
| `0x00` | Direct PWM                        |
| `0x01` | Safe direct PWM                   |
| `0x02` | Reserved for future velocity mode |
| `0x03` | Reserved                          |

### Semantic Notes
The command expresses motion intent. It does not guarantee that the same values are physically applied. Actual applied output should be reported separately through `MOTOR_STATE`.

### Typical Value Domain
Recommended operational range:

- `-255 .. +255`

---

## `STOP_CMD` (`0x11`)

### Role
Explicit stop request with semantic meaning stronger than a neutral motor command.

### Direction
- Host -> Controller

### Payload

| Field    | Size | Type    | Unit | Description                 |
| -------- | ---: | ------- | ---- | --------------------------- |
| `REASON` |    1 | `uint8` | enum | Reason for the stop request |

### `REASON` Values

|  Value | Meaning           |
| -----: | ----------------- |
| `0x00` | User request      |
| `0x01` | Host timeout      |
| `0x02` | Obstacle detected |
| `0x03` | Safety override   |
| `0x04` | Fault recovery    |
| `0x05` | Shutdown          |

### Semantic Notes
`STOP_CMD` should be interpreted as a supervisory or safety command, not merely as a motor target equal to zero.

---

## `MODE_CMD` (`0x12`)

### Role
Request a low-level mode transition.

### Direction
- Host -> Controller

### Payload

| Field      | Size | Type    | Unit | Description               |
| ---------- | ---: | ------- | ---- | ------------------------- |
| `MODE`     |    1 | `uint8` | enum | Requested controller mode |
| `RESERVED` |    1 | `uint8` | -    | Reserved, must be zero    |

### `MODE` Values

|  Value | Meaning                |
| -----: | ---------------------- |
| `0x00` | Idle                   |
| `0x01` | Manual / remote        |
| `0x02` | Autonomous             |
| `0x03` | Calibration            |
| `0x04` | Emergency stop latched |

### Semantic Notes
A mode changes the interpretation of future behavior and may enable or disable classes of commands.

---

## `GYRO_ZERO_CMD` (`0x13`)

### Role
Request gyroscope bias recalibration and integrated yaw reset.

### Direction
- Host -> Controller

### Payload
This message has no payload.

| Field | Size | Type | Description |
| ----- | ---: | ---- | ----------- |
| -     |    0 | -    | No payload  |

### Semantic Notes
This command alters low-level inertial state and should be treated as a critical state-changing operation.

---

## `CONFIG_SET` (`0x14`)

### Role
Generic configuration update message.

### Direction
- Host -> Controller

### Payload

| Field        |     Size | Type    | Unit    | Description                    |
| ------------ | -------: | ------- | ------- | ------------------------------ |
| `PARAM_ID`   |        1 | `uint8` | -       | Parameter identifier           |
| `VALUE_TYPE` |        1 | `uint8` | enum    | Declared encoding of the value |
| `VALUE`      | variable | bytes   | depends | Encoded value bytes            |

### `VALUE_TYPE` Examples

|  Value | Meaning  |
| -----: | -------- |
| `0x01` | `uint8`  |
| `0x02` | `int16`  |
| `0x03` | `uint16` |
| `0x04` | `int32`  |
| `0x05` | `uint32` |

### Semantic Notes
This message is intentionally generic. Its exact parameter catalogue is part of system configuration policy, not of core transport design.

---

## `HEARTBEAT_CMD` (`0x15`)

### Role
Host-originated keepalive frame.

### Direction
- Host -> Controller

### Payload

| Field          | Size | Type     | Unit     | Description                     |
| -------------- | ---: | -------- | -------- | ------------------------------- |
| `HOST_TIME_MS` |    4 | `uint32` | ms       | Host monotonic time             |
| `HOST_STATUS`  |    2 | `uint16` | bitfield | Optional host-side status flags |

### Semantic Notes
This message is useful when command traffic is sparse but watchdog supervision must remain active.

---

## Telemetry Messages

---

## `IMU_TELEMETRY` (`0x20`)

### Role
Report low-level inertial state.

### Direction
- Controller -> Host

### Payload

| Field             | Size | Type     | Unit   | Description                      |
| ----------------- | ---: | -------- | ------ | -------------------------------- |
| `MCU_TIME_MS`     |    4 | `uint32` | ms     | Controller timestamp             |
| `YAW_MRAD`        |    4 | `int32`  | mrad   | Integrated yaw angle             |
| `YAW_RATE_MRAD_S` |    4 | `int32`  | mrad/s | Angular velocity around yaw axis |
| `ACC_X_RAW`       |    2 | `int16`  | raw    | Accelerometer X raw value        |
| `ACC_Y_RAW`       |    2 | `int16`  | raw    | Accelerometer Y raw value        |
| `ACC_Z_RAW`       |    2 | `int16`  | raw    | Accelerometer Z raw value        |
| `GYRO_Z_RAW`      |    2 | `int16`  | raw    | Gyroscope Z raw value            |

### Total Payload Size
- `20 bytes`

### Interpretation Notes
This message provides both processed and raw inertial information.
The host may use processed yaw and yaw rate directly or use raw channels for later estimation strategies.

---

## `SAFETY_TELEMETRY` (`0x21`)

### Role
Report proximity-related and low-level safety-related sensor information.

### Direction
- Controller -> Host

### Payload

| Field          | Size | Type     | Unit     | Description          |
| -------------- | ---: | -------- | -------- | -------------------- |
| `MCU_TIME_MS`  |    4 | `uint32` | ms       | Controller timestamp |
| `ULTRA_CM`     |    2 | `uint16` | cm       | Ultrasonic distance  |
| `IR_LEFT_RAW`  |    2 | `uint16` | raw      | Left IR raw value    |
| `IR_RIGHT_RAW` |    2 | `uint16` | raw      | Right IR raw value   |
| `SAFETY_FLAGS` |    2 | `uint16` | bitfield | Safety status flags  |

### Total Payload Size
- `12 bytes`

### `SAFETY_FLAGS` Bit Definitions

|   Bit | Name                 | Meaning                         |
| ----: | -------------------- | ------------------------------- |
|     0 | `ULTRA_VALID`        | Ultrasonic value valid          |
|     1 | `IR_LEFT_ALERT`      | Left IR alert condition active  |
|     2 | `IR_RIGHT_ALERT`     | Right IR alert condition active |
|     3 | `FRONT_ALERT`        | Frontal obstacle alert active   |
|     4 | `CMD_TIMEOUT_ACTIVE` | Command timeout state active    |
|     5 | `EMERGENCY_STOP`     | Low-level emergency stop active |
|     6 | `RESERVED`           | Reserved                        |
|     7 | `RESERVED`           | Reserved                        |
| 8..15 | `RESERVED`           | Reserved                        |

### Interpretation Notes
This message communicates both raw measurements and the controller's local safety interpretation.

---

## `ENCODER_TELEMETRY` (`0x22`)

### Role
Report wheel encoder state.

### Direction
- Controller -> Host

### Payload

| Field         | Size | Type     | Unit     | Description                    |
| ------------- | ---: | -------- | -------- | ------------------------------ |
| `MCU_TIME_MS` |    4 | `uint32` | ms       | Controller timestamp           |
| `TICKS_LEFT`  |    4 | `int32`  | ticks    | Cumulative left encoder count  |
| `TICKS_RIGHT` |    4 | `int32`  | ticks    | Cumulative right encoder count |
| `DT_MS`       |    2 | `uint16` | ms       | Encoder update interval        |
| `ENC_FLAGS`   |    2 | `uint16` | bitfield | Encoder status flags           |

### Total Payload Size
- `16 bytes`

### `ENC_FLAGS` Bit Definitions

|   Bit | Name            | Meaning                              |
| ----: | --------------- | ------------------------------------ |
|     0 | `LEFT_VALID`    | Left encoder reading valid           |
|     1 | `RIGHT_VALID`   | Right encoder reading valid          |
|     2 | `LEFT_DIR_NEG`  | Left wheel direction negative        |
|     3 | `RIGHT_DIR_NEG` | Right wheel direction negative       |
|     4 | `OVERFLOW_WARN` | Counter rollover or anomaly detected |
|     5 | `RESERVED`      | Reserved                             |
| 6..15 | `RESERVED`      | Reserved                             |

### Interpretation Notes
RSP-v1 conceptually prefers cumulative counts over per-frame deltas because they are more resilient to packet loss and easier to process on the host.

---

## `MOTOR_STATE` (`0x23`)

### Role
Report current low-level motor actuation state.

### Direction
- Controller -> Host

### Payload

| Field               | Size | Type     | Unit      | Description           |
| ------------------- | ---: | -------- | --------- | --------------------- |
| `MCU_TIME_MS`       |    4 | `uint32` | ms        | Controller timestamp  |
| `TARGET_PWM_LEFT`   |    2 | `int16`  | PWM units | Left target command   |
| `TARGET_PWM_RIGHT`  |    2 | `int16`  | PWM units | Right target command  |
| `CURRENT_PWM_LEFT`  |    2 | `int16`  | PWM units | Left applied command  |
| `CURRENT_PWM_RIGHT` |    2 | `int16`  | PWM units | Right applied command |
| `MOTOR_FLAGS`       |    2 | `uint16` | bitfield  | Motor status flags    |

### Total Payload Size
- `14 bytes`

### `MOTOR_FLAGS` Bit Definitions

|   Bit | Name             | Meaning                          |
| ----: | ---------------- | -------------------------------- |
|     0 | `MOTORS_ENABLED` | Motor driver active              |
|     1 | `STBY_HIGH`      | Driver standby released          |
|     2 | `CMD_TIMEOUT`    | Timeout condition active         |
|     3 | `SLEW_LIMITING`  | Output ramp limiting active      |
|     4 | `STOP_REQUESTED` | Stop condition currently imposed |
|     5 | `RESERVED`       | Reserved                         |
| 6..15 | `RESERVED`       | Reserved                         |

### Interpretation Notes
This message should be interpreted as actual low-level actuation state, not merely command intent.

---

## `HEARTBEAT_STATE` (`0x24`)

### Role
Report compact low-level system health and readiness.

### Direction
- Controller -> Host

### Payload

| Field          | Size | Type     | Unit     | Description                       |
| -------------- | ---: | -------- | -------- | --------------------------------- |
| `MCU_TIME_MS`  |    4 | `uint32` | ms       | Controller timestamp              |
| `UPTIME_S`     |    2 | `uint16` | s        | Controller uptime                 |
| `STATUS_FLAGS` |    2 | `uint16` | bitfield | Global readiness and health flags |
| `FW_MAJOR`     |    1 | `uint8`  | -        | Firmware major version            |
| `FW_MINOR`     |    1 | `uint8`  | -        | Firmware minor version            |
| `ERROR_CODE`   |    2 | `uint16` | enum     | Latched error code                |

### Total Payload Size
- `12 bytes`

### `STATUS_FLAGS` Bit Definitions

|   Bit | Name             | Meaning                    |
| ----: | ---------------- | -------------------------- |
|     0 | `IMU_READY`      | IMU subsystem ready        |
|     1 | `ULTRA_READY`    | Ultrasonic subsystem ready |
|     2 | `IR_READY`       | IR subsystem ready         |
|     3 | `ENCODERS_READY` | Encoder subsystem ready    |
|     4 | `MOTORS_READY`   | Motor driver ready         |
|     5 | `CALIBRATING`    | Calibration in progress    |
|     6 | `FAULT_LATCHED`  | Fault currently latched    |
|     7 | `HOST_LINK_OK`   | Host communication healthy |
| 8..15 | `RESERVED`       | Reserved                   |

### Interpretation Notes
This message is intended as a compact health summary rather than a detailed telemetry packet.

---

## Standard Error Categories

These error categories are used by the `ERROR` message and may also be referenced by `HEARTBEAT_STATE`.

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
| `0x0D` | `NOT_IMPLEMENTED`      | Recognized but not implemented                     |

---

## ACK Guidance by Message

### Usually ACKed

| Message         | Typical ACK Policy      |
| --------------- | ----------------------- |
| `PING`          | Yes, during diagnostics |
| `STOP_CMD`      | Yes                     |
| `MODE_CMD`      | Yes                     |
| `GYRO_ZERO_CMD` | Yes                     |
| `CONFIG_SET`    | Yes                     |

### Usually Not ACKed

| Message             | Typical ACK Policy             |
| ------------------- | ------------------------------ |
| `MOTOR_CMD`         | No during continuous streaming |
| `IMU_TELEMETRY`     | No                             |
| `SAFETY_TELEMETRY`  | No                             |
| `ENCODER_TELEMETRY` | No                             |
| `MOTOR_STATE`       | No                             |
| `HEARTBEAT_STATE`   | No                             |
| `HEARTBEAT_CMD`     | No                             |

---

## Units Summary

| Quantity            | Unit        |
| ------------------- | ----------- |
| Controller time     | `ms`        |
| Host time           | `ms`        |
| Uptime              | `s`         |
| Yaw angle           | `mrad`      |
| Yaw rate            | `mrad/s`    |
| Ultrasonic distance | `cm`        |
| Encoder position    | `ticks`     |
| PWM command         | `PWM units` |
| Raw IR / IMU values | `raw`       |

---

## Design Summary

The payload layer of RSP-v1 follows a few key ideas:

- commands express intent
- telemetry reports state
- actual actuation state is separate from requested actuation
- all physical quantities use explicit units
- flags represent orthogonal boolean state compactly
- cumulative encoder counts are preferred to deltas
- payload meaning must remain stable across the protocol version

This note should be read together with the main protocol note:

- **`Robot Serial Protocol v1 (RSP-v1)`**
