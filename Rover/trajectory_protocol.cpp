#include <AP_HAL/AP_HAL_Boards.h>
#include <stdint.h>

#include "config.h"

#if MODE_TRAJECTORY_ENABLED

#include "trajectory_protocol.h"

#include <cstring>

static constexpr uint8_t TRAJECTORY_HEADER_LENGTH = 8;
static constexpr uint8_t TRAJECTORY_POINT_LENGTH = TRAJECTORY_HEADER_LENGTH + 4 + 6 * sizeof(float);
static constexpr uint8_t TRAJECTORY_FINALISE_LENGTH = TRAJECTORY_HEADER_LENGTH + 2;
static constexpr uint8_t TRAJECTORY_ACK_LENGTH = 20;

uint16_t TrajectoryProtocol::read_u16(const uint8_t *data)
{
    return static_cast<uint16_t>(data[0]) |
           static_cast<uint16_t>(data[1]) << 8U;
}

uint32_t TrajectoryProtocol::read_u32(const uint8_t *data)
{
    return static_cast<uint32_t>(data[0]) |
           static_cast<uint32_t>(data[1]) << 8U |
           static_cast<uint32_t>(data[2]) << 16U |
           static_cast<uint32_t>(data[3]) << 24U;
}

float TrajectoryProtocol::read_float(const uint8_t *data)
{
    const uint32_t bits = read_u32(data);
    float value;
    memcpy(&value, &bits, sizeof(value));
    return value;
}

void TrajectoryProtocol::write_u16(uint8_t *data, uint16_t value)
{
    data[0] = value & 0xffU;
    data[1] = value >> 8U;
}

void TrajectoryProtocol::write_u32(uint8_t *data, uint32_t value)
{
    data[0] = value & 0xffU;
    data[1] = value >> 8U;
    data[2] = value >> 16U;
    data[3] = value >> 24U;
}

void TrajectoryProtocol::write_float(uint8_t *data, float value)
{
    uint32_t bits;
    memcpy(&bits, &value, sizeof(bits));
    write_u32(data, bits);
}

TrajectoryProtocol::Error TrajectoryProtocol::decode_request(const uint8_t *payload, uint8_t payload_length,
                                                              Request &request)
{
    if (payload == nullptr || payload_length < TRAJECTORY_HEADER_LENGTH) {
        return Error::INVALID_LENGTH;
    }

    request.opcode = static_cast<Opcode>(payload[1]);
    request.sequence = read_u16(&payload[2]);
    request.trajectory_id = read_u32(&payload[4]);
    request.point_index = 0;
    request.point_count = 0;
    request.point = {};

    if (payload[0] != VERSION) {
        return Error::INVALID_VERSION;
    }

    switch (request.opcode) {
    case Opcode::CLEAR:
    case Opcode::START:
    case Opcode::STOP:
    case Opcode::STATUS:
        return payload_length == TRAJECTORY_HEADER_LENGTH ? Error::NONE : Error::INVALID_LENGTH;

    case Opcode::POINT:
        if (payload_length != TRAJECTORY_POINT_LENGTH) {
            return Error::INVALID_LENGTH;
        }
        request.point_index = read_u16(&payload[8]);
        request.point_count = read_u16(&payload[10]);
        request.point.time_s = read_float(&payload[12]);
        request.point.x_m = read_float(&payload[16]);
        request.point.y_m = read_float(&payload[20]);
        request.point.yaw_rad = read_float(&payload[24]);
        request.point.speed_mps = read_float(&payload[28]);
        request.point.yaw_rate_radps = read_float(&payload[32]);
        return Error::NONE;

    case Opcode::FINALISE:
        if (payload_length != TRAJECTORY_FINALISE_LENGTH) {
            return Error::INVALID_LENGTH;
        }
        request.point_count = read_u16(&payload[8]);
        return Error::NONE;

    case Opcode::ACK:
    default:
        return Error::INVALID_OPCODE;
    }
}

uint8_t TrajectoryProtocol::encode_ack(const Ack &ack, uint8_t *payload, uint8_t payload_capacity)
{
    if (payload == nullptr || payload_capacity < TRAJECTORY_ACK_LENGTH) {
        return 0;
    }

    payload[0] = VERSION;
    payload[1] = static_cast<uint8_t>(Opcode::ACK);
    write_u16(&payload[2], ack.sequence);
    write_u32(&payload[4], ack.trajectory_id);
    payload[8] = static_cast<uint8_t>(ack.request_opcode);
    payload[9] = static_cast<uint8_t>(ack.error);
    payload[10] = static_cast<uint8_t>(ack.state);
    payload[11] = 0;
    write_u16(&payload[12], ack.received_count);
    write_u16(&payload[14], ack.expected_count);
    write_float(&payload[16], ack.duration_s);
    return TRAJECTORY_ACK_LENGTH;
}

#endif // MODE_TRAJECTORY_ENABLED
