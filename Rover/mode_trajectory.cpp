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
    _closest_trajectory_index = 0;
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

bool ModeTrajectory::apply_output()
{
    if (_control_output.wheel_rate_output) {
        const float wheel_rate_max = g2.wheel_rate_control.get_rate_max_rads();
        if (!g2.motors.have_skid_steering() ||
            !g2.wheel_rate_control.enabled(0) ||
            !g2.wheel_rate_control.enabled(1) ||
            !is_positive(wheel_rate_max)) {
            return false;
        }

        const float left_rate_pct = constrain_float(
            100.0f * _control_output.wheel_rate_left_cmd_radps / wheel_rate_max,
            -100.0f, 100.0f);
        const float right_rate_pct = constrain_float(
            100.0f * _control_output.wheel_rate_right_cmd_radps / wheel_rate_max,
            -100.0f, 100.0f);
        const float throttle_pct = 0.5f * (left_rate_pct + right_rate_pct);
        const float steering = 22.5f * (left_rate_pct - right_rate_pct);
        g2.motors.set_steering(steering, false);
        g2.motors.set_throttle(throttle_pct);
        return true;
    }

    calc_steering_from_turn_rate(_control_output.yaw_rate_cmd_radps);
    calc_throttle(_control_output.speed_cmd_mps, false);
    return true;
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
        if (_have_control_output && !apply_output()) {
            stop_with_warning("wheel-rate output unavailable");
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

    const uint32_t now_ms = AP_HAL::millis();
    if (g2.wheel_encoder.num_sensors() >= 2) {
        state.wheel_rate_left_radps = g2.wheel_encoder.get_rate(0);
        state.wheel_rate_right_radps = g2.wheel_encoder.get_rate(1);
        state.wheel_left_time_ms = g2.wheel_encoder.get_last_reading_ms(0);
        state.wheel_right_time_ms = g2.wheel_encoder.get_last_reading_ms(1);
        state.wheel_left_valid = g2.wheel_encoder.healthy(0) &&
                                 now_ms - state.wheel_left_time_ms <= AP_WHEEL_RATE_CONTROL_TIMEOUT_MS;
        state.wheel_right_valid = g2.wheel_encoder.healthy(1) &&
                                  now_ms - state.wheel_right_time_ms <= AP_WHEEL_RATE_CONTROL_TIMEOUT_MS;
    }

    const bool wheel_measurements_valid = state.wheel_left_valid && state.wheel_right_valid;
    const bool wheel_rate_output_available = wheel_measurements_valid &&
                                             g2.motors.have_skid_steering() &&
                                             g2.wheel_rate_control.enabled(0) &&
                                             g2.wheel_rate_control.enabled(1) &&
                                             is_positive(g2.wheel_rate_control.get_rate_max_rads());
    rover.tidal_control.set_use_residual_smo(wheel_measurements_valid);
    rover.tidal_control.set_use_wheel_compensation(wheel_rate_output_available);
    rover.tidal_control.set_wheel_rate_limit(g2.wheel_rate_control.get_rate_max_rads());

    float progress;
    if (!rover.trajectory.closest_progress(state.x_m, state.y_m,
                                           _closest_trajectory_index,
                                           _closest_trajectory_index,
                                           progress)) {
        stop_with_warning("trajectory progress invalid");
        return;
    }
    if (!rover.tidal_control.update(state, _reference, progress, _control_output) ||
        !_control_output.valid || _control_output.stop_required) {
        stop_with_warning("control output invalid");
        return;
    }

    _have_control_output = true;
    if (!apply_output()) {
        stop_with_warning("wheel-rate output unavailable");
    }
}

#endif // MODE_TRAJECTORY_ENABLED
