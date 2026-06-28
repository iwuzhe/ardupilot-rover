#pragma once

#include <AR_Trajectory/AR_Trajectory.h>
#include <stdint.h>

class TrajectoryProtocol {
public:
    static constexpr uint8_t VERSION = 1;
    static constexpr uint16_t MAVLINK_PAYLOAD_TYPE = 32768;
    static constexpr uint8_t MAX_PAYLOAD_LENGTH = 128;
    static constexpr uint32_t UPLOAD_TIMEOUT_MS = 5000;

    enum class Opcode : uint8_t {
        CLEAR = 1,
        POINT = 2,
        FINALISE = 3,
        START = 4,
        STOP = 5,
        STATUS = 6,
        ACK = 128,
    };

    enum class Error : uint8_t {
        NONE,
        INVALID_VERSION,
        INVALID_LENGTH,
        INVALID_OPCODE,
        INVALID_STATE,
        INVALID_ID,
        INVALID_INDEX,
        INVALID_COUNT,
        INVALID_POINT,
        DUPLICATE_CONFLICT,
        MISSING_POINT,
        INVALID_TIME_AXIS,
        NOT_READY,
        NOT_ARMED,
        MODE_CHANGE_FAILED,
        UPLOAD_TIMEOUT,
    };

    struct Request {
        Opcode opcode;
        uint16_t sequence;
        uint32_t trajectory_id;
        uint16_t point_index;
        uint16_t point_count;
        AR_TrajectoryPoint point;
    };

    struct Ack {
        uint16_t sequence;
        uint32_t trajectory_id;
        Opcode request_opcode;
        Error error;
        AR_Trajectory::State state;
        uint16_t received_count;
        uint16_t expected_count;
        float duration_s;
    };

    static Error decode_request(const uint8_t *payload, uint8_t payload_length, Request &request);
    static uint8_t encode_ack(const Ack &ack, uint8_t *payload, uint8_t payload_capacity);

private:
    static uint16_t read_u16(const uint8_t *data);
    static uint32_t read_u32(const uint8_t *data);
    static float read_float(const uint8_t *data);
    static void write_u16(uint8_t *data, uint16_t value);
    static void write_u32(uint8_t *data, uint32_t value);
    static void write_float(uint8_t *data, float value);
};
