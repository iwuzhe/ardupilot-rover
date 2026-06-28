#include "AR_Trajectory.h"

#include <AP_Math/AP_Math.h>
#include <cmath>
#include <cstring>

AR_Trajectory::AR_Trajectory()
{
    clear();
}

void AR_Trajectory::clear(uint32_t trajectory_id)
{
    memset(_received_mask, 0, sizeof(_received_mask));
    _start_time_us = 0;
    _trajectory_id = trajectory_id;
    _expected_count = 0;
    _received_count = 0;
    _sample_index = 0;
    _state = State::EMPTY;
    _finalised = false;
}

bool AR_Trajectory::point_is_valid(const AR_TrajectoryPoint &point)
{
    return isfinite(point.time_s) && !is_negative(point.time_s) &&
           isfinite(point.x_m) && isfinite(point.y_m) &&
           isfinite(point.yaw_rad) && isfinite(point.speed_mps) &&
           isfinite(point.yaw_rate_radps);
}

bool AR_Trajectory::point_received(uint16_t index) const
{
    return (_received_mask[index / 32U] & (1UL << (index % 32U))) != 0;
}

void AR_Trajectory::set_point_received(uint16_t index)
{
    _received_mask[index / 32U] |= 1UL << (index % 32U);
}

bool AR_Trajectory::point_matches(uint16_t index, const AR_TrajectoryPoint &point) const
{
    const AR_TrajectoryPoint &stored = _points[index];
    return memcmp(&stored, &point, sizeof(point)) == 0;
}

AR_Trajectory::Result AR_Trajectory::set_point(uint32_t trajectory_id, uint16_t index, uint16_t total_count,
                                               const AR_TrajectoryPoint &point)
{
    if (_state != State::EMPTY && _state != State::LOADING) {
        return Result::INVALID_STATE;
    }
    if (trajectory_id == 0) {
        return Result::INVALID_ID;
    }
    if (total_count < 2 || total_count > AR_TRAJECTORY_MAX_POINTS) {
        return Result::INVALID_COUNT;
    }
    if (index >= total_count) {
        return Result::INVALID_INDEX;
    }
    if (!point_is_valid(point)) {
        return Result::INVALID_POINT;
    }

    if (_state == State::EMPTY) {
        if (_trajectory_id != 0 && _trajectory_id != trajectory_id) {
            return Result::INVALID_ID;
        }
        _trajectory_id = trajectory_id;
        _expected_count = total_count;
        _state = State::LOADING;
    } else if (_trajectory_id != trajectory_id) {
        return Result::INVALID_ID;
    } else if (_expected_count != total_count) {
        return Result::INVALID_COUNT;
    }

    if (point_received(index)) {
        return point_matches(index, point) ? Result::OK : Result::DUPLICATE_CONFLICT;
    }

    _points[index] = point;
    set_point_received(index);
    _received_count++;
    return Result::OK;
}

AR_Trajectory::Result AR_Trajectory::finalise(uint32_t trajectory_id, uint16_t expected_count)
{
    if (_state != State::LOADING) {
        return Result::INVALID_STATE;
    }
    if (_trajectory_id != trajectory_id) {
        return Result::INVALID_ID;
    }
    if (expected_count != _expected_count || expected_count < 2) {
        return Result::INVALID_COUNT;
    }
    if (_received_count != _expected_count) {
        return Result::MISSING_POINT;
    }

    for (uint16_t i = 0; i < _expected_count; i++) {
        if (!point_received(i)) {
            return Result::MISSING_POINT;
        }
        if (i > 0 && !(_points[i].time_s > _points[i - 1].time_s)) {
            _state = State::ERROR;
            return Result::INVALID_TIME_AXIS;
        }
    }

    _finalised = true;
    _state = State::READY;
    return Result::OK;
}

AR_Trajectory::Result AR_Trajectory::start(uint64_t now_us)
{
    if (_state == State::RUNNING) {
        return Result::INVALID_STATE;
    }
    if (!_finalised || _expected_count < 2) {
        return Result::NOT_READY;
    }
    _start_time_us = now_us;
    _sample_index = 0;
    _state = State::RUNNING;
    return Result::OK;
}

void AR_Trajectory::stop()
{
    if (_finalised) {
        _state = State::STOPPED;
    } else {
        _state = State::EMPTY;
    }
}

AR_Trajectory::Result AR_Trajectory::sample(uint64_t now_us, AR_TrajectoryReference &reference)
{
    if (_state != State::RUNNING) {
        return Result::INVALID_STATE;
    }
    if (now_us < _start_time_us) {
        return Result::INVALID_STATE;
    }

    const float elapsed_s = (now_us - _start_time_us) * 1.0e-6f;
    const AR_TrajectoryPoint &last = _points[_expected_count - 1];
    if (elapsed_s >= last.time_s) {
        static_cast<AR_TrajectoryPoint &>(reference) = last;
        reference.segment_index = _expected_count - 2;
        _state = State::FINISHED;
        return Result::OK;
    }

    while ((_sample_index + 1U) < (_expected_count - 1U) &&
           elapsed_s > _points[_sample_index + 1U].time_s) {
        _sample_index++;
    }

    const AR_TrajectoryPoint &p0 = _points[_sample_index];
    const AR_TrajectoryPoint &p1 = _points[_sample_index + 1U];
    const float alpha = constrain_float((elapsed_s - p0.time_s) / (p1.time_s - p0.time_s), 0.0f, 1.0f);

    reference.time_s = elapsed_s;
    reference.x_m = linear_interpolate(p0.x_m, p1.x_m, alpha, 0.0f, 1.0f);
    reference.y_m = linear_interpolate(p0.y_m, p1.y_m, alpha, 0.0f, 1.0f);
    reference.yaw_rad = wrap_PI(p0.yaw_rad + alpha * wrap_PI(p1.yaw_rad - p0.yaw_rad));
    reference.speed_mps = linear_interpolate(p0.speed_mps, p1.speed_mps, alpha, 0.0f, 1.0f);
    reference.yaw_rate_radps = linear_interpolate(p0.yaw_rate_radps, p1.yaw_rate_radps, alpha, 0.0f, 1.0f);
    reference.segment_index = _sample_index;
    return Result::OK;
}

float AR_Trajectory::duration_s() const
{
    if (!_finalised || _expected_count == 0) {
        return 0.0f;
    }
    return _points[_expected_count - 1].time_s;
}
