# Rover trajectory upload protocol

This experimental protocol carries time-indexed Rover trajectories in the
MAVLink `TUNNEL` message. It uses payload type `32768`, which is reserved for
local experiments. It must be replaced by a registered MAVLink payload type
before the feature is proposed for upstream release.

All multi-byte fields use little-endian byte order. Requests start with this
eight-byte header:

| Offset | Type | Field |
|---:|---|---|
| 0 | `uint8_t` | Protocol version, currently 1 |
| 1 | `uint8_t` | Opcode |
| 2 | `uint16_t` | Request sequence |
| 4 | `uint32_t` | Trajectory ID |

The sender must target the autopilot system and component explicitly. Broadcast
requests are ignored. A trajectory upload expires after five seconds without a
successfully accepted point. Trajectory ID zero is reserved and cannot be used
for an upload.

## Opcodes

| Value | Name | Payload after header |
|---:|---|---|
| 1 | `CLEAR` | None |
| 2 | `POINT` | Point index, point count, then six `float` values |
| 3 | `FINALISE` | Expected point count as `uint16_t` |
| 4 | `START` | None |
| 5 | `STOP` | None |
| 6 | `STATUS` | None |
| 128 | `ACK` | Response only |

The `POINT` values are `time_s`, `x_m`, `y_m`, `yaw_rad`, `speed_mps`, and
`yaw_rate_radps`. The first two fields are `uint16_t`. The total point payload,
including the common header, is 36 bytes.

Coordinates use the ArduPilot local NED frame: `x_m` is north and `y_m` is
east relative to the EKF origin. Yaw and yaw rate follow ArduPilot's NED sign
convention. The companion computer must convert ENU or counter-clockwise source
data before upload.

## ACK payload

Every recognised request receives a 20-byte ACK containing the protocol
version, ACK opcode, request sequence, active trajectory ID, request opcode,
error, trajectory state, one reserved byte, received count, expected count,
and trajectory duration as a `float`.

Repeated points with identical values are accepted without increasing the
received count. A repeated point with different values is rejected. FINALISE
rejects missing points and non-increasing timestamps. START requires a complete
trajectory and an armed vehicle. STOP leaves the trajectory data loaded and
changes an active trajectory mode to HOLD.
