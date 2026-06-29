#include "Rover.h"

#if MODE_TRAJECTORY_ENABLED

bool ModeTrajectory::_enter()
{
    if (!rover.arming.is_armed() || !rover.trajectory.loaded()) {
        return false;
    }

    Vector2f reset;
    float yaw_reset;
    _position_reset_ms = ahrs.getLastPosNorthEastReset(reset);
    _yaw_reset_ms = ahrs.getLastYawResetAngle(yaw_reset);
    _failure_reported = false;
    _have_control_output = false;
    _control_output = {};
    rover.tidal_control.reset();

    const uint64_t now_us = AP_HAL::micros64();
    if (rover.trajectory.start(now_us) != AR_Trajectory::Result::OK) {
        return false;
    }
    _next_control_us = now_us;
    _last_control_us = 0;
    set_zero_output();
    return true;
}

void ModeTrajectory::_exit()
{
    rover.trajectory.stop();
    rover.tidal_control.reset();
    _have_control_output = false;
    set_zero_output();
}

void ModeTrajectory::set_zero_output()
{
    g2.motors.set_throttle(0.0f);
    g2.motors.set_steering(0.0f);
}

void ModeTrajectory::apply_output()
{
    calc_steering_from_turn_rate(_control_output.yaw_rate_cmd_radps);
    calc_throttle(_control_output.speed_cmd_mps, false);
}

void ModeTrajectory::stop_with_warning(const char *reason)
{
    rover.trajectory.stop();
    rover.tidal_control.reset();
    _have_control_output = false;
    set_zero_output();
    if (!_failure_reported) {
        _failure_reported = true;
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "Trajectory stopped: %s", reason);
    }
}

void ModeTrajectory::update()
{
    if (!rover.trajectory.running()) {
        set_zero_output();
        return;
    }

    Vector2f position;
    Vector3f velocity;
    if (!rover.ekf_position_ok() ||
        !ahrs.get_relative_position_NE_origin(position) ||
        !ahrs.get_velocity_NED(velocity)) {
        stop_with_warning("navigation state invalid");
        return;
    }

    Vector2f position_reset;
    float yaw_reset;
    const uint32_t position_reset_ms = ahrs.getLastPosNorthEastReset(position_reset);
    const uint32_t yaw_reset_ms = ahrs.getLastYawResetAngle(yaw_reset);
    if (position_reset_ms != _position_reset_ms || yaw_reset_ms != _yaw_reset_ms) {
        stop_with_warning("navigation origin reset");
        return;
    }

    const uint64_t now_us = AP_HAL::micros64();
    if (now_us < _next_control_us) {
        if (_have_control_output) {
            apply_output();
        }
        return;
    }
    _next_control_us = now_us + CONTROL_INTERVAL_US;

    const float dt_s = (_last_control_us == 0) ?
                       CONTROL_INTERVAL_US * 1.0e-6f :
                       (now_us - _last_control_us) * 1.0e-6f;
    _last_control_us = now_us;

    if (rover.trajectory.sample(now_us, _reference) != AR_Trajectory::Result::OK) {
        stop_with_warning("trajectory sampling failed");
        return;
    }

    if (rover.trajectory.finished()) {
        _have_control_output = false;
        set_zero_output();
        return;
    }

    const float yaw_rad = ahrs.get_yaw();
    AR_TidalState state {};
    state.x_m = position.x;
    state.y_m = position.y;
    state.yaw_rad = yaw_rad;
    state.speed_mps = velocity.x * cosf(yaw_rad) + velocity.y * sinf(yaw_rad);
    state.yaw_rate_radps = ahrs.get_yaw_rate_earth();
    state.dt_s = dt_s;
    state.position_valid = true;
    state.velocity_valid = true;

    const float duration_s = rover.trajectory.duration_s();
    const float progress = is_positive(duration_s) ? _reference.time_s / duration_s : 0.0f;
    if (!rover.tidal_control.update(state, _reference, progress, _control_output) ||
        !_control_output.valid || _control_output.stop_required) {
        stop_with_warning("control output invalid");
        return;
    }

    _have_control_output = true;
    apply_output();
}

#endif // MODE_TRAJECTORY_ENABLED
