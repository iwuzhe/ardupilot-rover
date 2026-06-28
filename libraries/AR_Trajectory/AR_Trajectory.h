#pragma once

#include "AR_Trajectory_config.h"
#include "AR_Trajectory_Types.h"

#include <AP_Common/AP_Common.h>
#include <stdint.h>

class AR_Trajectory {
public:
    static_assert(AR_TRAJECTORY_MAX_POINTS >= 2, "AR_Trajectory requires at least two points");
    static_assert(AR_TRAJECTORY_MAX_POINTS <= UINT16_MAX, "AR_Trajectory point count exceeds protocol capacity");

    enum class State : uint8_t {
        EMPTY,
        LOADING,
        READY,
        RUNNING,
        FINISHED,
        STOPPED,
        ERROR,
    };

    enum class Result : uint8_t {
        OK,
        INVALID_STATE,
        INVALID_ID,
        INVALID_INDEX,
        INVALID_COUNT,
        INVALID_POINT,
        DUPLICATE_CONFLICT,
        MISSING_POINT,
        INVALID_TIME_AXIS,
        NOT_READY,
    };

    AR_Trajectory();

    CLASS_NO_COPY(AR_Trajectory);

    void clear(uint32_t trajectory_id = 0);
    Result set_point(uint32_t trajectory_id, uint16_t index, uint16_t total_count, const AR_TrajectoryPoint &point);
    Result finalise(uint32_t trajectory_id, uint16_t expected_count);
    Result start(uint64_t now_us);
    void stop();
    Result sample(uint64_t now_us, AR_TrajectoryReference &reference);

    bool loaded() const { return _finalised; }
    bool running() const { return _state == State::RUNNING; }
    bool finished() const { return _state == State::FINISHED; }
    State state() const { return _state; }
    uint32_t trajectory_id() const { return _trajectory_id; }
    uint16_t received_count() const { return _received_count; }
    uint16_t expected_count() const { return _expected_count; }
    float duration_s() const;

private:
    static constexpr uint16_t RECEIVED_WORD_COUNT = (AR_TRAJECTORY_MAX_POINTS + 31U) / 32U;

    static bool point_is_valid(const AR_TrajectoryPoint &point);
    bool point_received(uint16_t index) const;
    void set_point_received(uint16_t index);
    bool point_matches(uint16_t index, const AR_TrajectoryPoint &point) const;

    AR_TrajectoryPoint _points[AR_TRAJECTORY_MAX_POINTS] {};
    uint32_t _received_mask[RECEIVED_WORD_COUNT] {};
    uint64_t _start_time_us = 0;
    uint32_t _trajectory_id = 0;
    uint16_t _expected_count = 0;
    uint16_t _received_count = 0;
    uint16_t _sample_index = 0;
    State _state = State::EMPTY;
    bool _finalised = false;
};
