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

    const uint64_t now_us = AP_HAL::micros64();
    if (rover.trajectory.start(now_us) != AR_Trajectory::Result::OK) {
        return false;
    }
    _next_control_us = now_us;
    set_zero_output();
    return true;
}

void ModeTrajectory::_exit()
{
    rover.trajectory.stop();
    set_zero_output();
}

void ModeTrajectory::set_zero_output()
{
    g2.motors.set_throttle(0.0f);
    g2.motors.set_steering(0.0f);
}

void ModeTrajectory::stop_with_warning(const char *reason)
{
    rover.trajectory.stop();
    set_zero_output();
    if (!_failure_reported) {
        _failure_reported = true;
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "Trajectory stopped: %s", reason);
    }
}

void ModeTrajectory::update()
{
    // Stage 6 deliberately produces no effective motor command.
    set_zero_output();

    if (!rover.trajectory.running()) {
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
        return;
    }
    _next_control_us = now_us + CONTROL_INTERVAL_US;

    if (rover.trajectory.sample(now_us, _reference) != AR_Trajectory::Result::OK) {
        stop_with_warning("trajectory sampling failed");
        return;
    }

    // AR_TidalControl is introduced in stage 7. Until then, the sampled
    // reference is retained for inspection while motor outputs remain zero.
}

#endif // MODE_TRAJECTORY_ENABLED
