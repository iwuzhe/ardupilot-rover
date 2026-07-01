#include "Rover.h"

#if MODE_TRAJECTORY_ENABLED

bool ModeTrajectory::_enter()
{
    if (!rover.arming.is_armed() || !rover.trajectory.loaded() ||
        !rover.ekf_position_ok()) {
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
    _terminal_complete = false;
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
    _terminal_complete = false;
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

void ModeTrajectory::write_log(const AR_TidalState &state, float progress) const
{
#if HAL_LOGGING_ENABLED
    if (g2.traj_log_enable == 0) {
        return;
    }

    const uint64_t time_us = AP_HAL::micros64();
    const AR_BacksteppingPPCDebug &debug = rover.tidal_control.backstepping_debug();
    const AR_SlipEstimate &slip = rover.tidal_control.slip_estimate();
    const uint8_t control_flags = (debug.ppc_active ? 1U : 0U) |
                                  (debug.ppc_fallback ? 2U : 0U) |
                                  (_control_output.ppc_violation ? 4U : 0U) |
                                  (debug.ppc_slip_scheduled ? 8U : 0U);
    const uint8_t slip_flags = (slip.valid ? 1U : 0U) |
                               (slip.new_data ? 2U : 0U) |
                               (state.wheel_left_valid ? 4U : 0U) |
                               (state.wheel_right_valid ? 8U : 0U);
    const uint8_t wheel_flags = (_control_output.wheel_rate_output ? 1U : 0U) |
                                (_control_output.slip_compensation_applied ? 2U : 0U);

    // @LoggerMessage: TRJR
    // @Description: Trajectory mode sampled reference
    // @Field: TimeUS: Time since system startup
    // @Field: Seg: Trajectory segment index
    // @Field: TRef: Reference trajectory time
    // @Field: XRef: Reference north position
    // @Field: YRef: Reference east position
    // @Field: YawRef: Reference yaw
    // @Field: SpdRef: Reference forward speed
    // @Field: YRRef: Reference yaw rate
    // @Field: Prog: Spatial trajectory progress
    AP::logger().WriteStreaming("TRJR", "TimeUS,Seg,TRef,XRef,YRef,YawRef,SpdRef,YRRef,Prog",
                                "QHfffffff", time_us, _reference.segment_index,
                                _reference.time_s, _reference.x_m, _reference.y_m,
                                _reference.yaw_rad, _reference.speed_mps,
                                _reference.yaw_rate_radps, progress);

    // @LoggerMessage: TRJC
    // @Description: Trajectory mode controller state and output
    // @Field: TimeUS: Time since system startup
    // @Field: X: Current north position
    // @Field: Y: Current east position
    // @Field: Yaw: Current yaw
    // @Field: EX: Body longitudinal position error
    // @Field: EY: Body lateral position error
    // @Field: EYC: PPC lateral control error
    // @Field: EYaw: Wrapped yaw error
    // @Field: Rho: PPC lateral boundary
    // @Field: VCmd: Forward speed command
    // @Field: WCmd: Yaw-rate command
    // @Field: Flags: PPC active, fallback, violation and slip-scheduled flags
    AP::logger().WriteStreaming("TRJC", "TimeUS,X,Y,Yaw,EX,EY,EYC,EYaw,Rho,VCmd,WCmd,Flags",
                                "QffffffffffB", time_us, state.x_m, state.y_m,
                                state.yaw_rad, debug.error_x_m, debug.error_y_m,
                                debug.error_y_control_m, debug.error_yaw_rad,
                                debug.ppc_rho_m, _control_output.speed_cmd_mps,
                                _control_output.yaw_rate_cmd_radps, control_flags);

    // @LoggerMessage: TRJS
    // @Description: Trajectory mode wheel measurements and Residual-SMO estimate
    // @Field: TimeUS: Time since system startup
    // @Field: WL: Left wheel angular rate
    // @Field: WR: Right wheel angular rate
    // @Field: ResL: Left kinematic residual
    // @Field: ResR: Right kinematic residual
    // @Field: SlipL: Estimated left slip ratio
    // @Field: SlipR: Estimated right slip ratio
    // @Field: Conf: Slip estimate confidence
    // @Field: Flags: Estimate valid, new data and wheel validity flags
    AP::logger().WriteStreaming("TRJS", "TimeUS,WL,WR,ResL,ResR,SlipL,SlipR,Conf,Flags",
                                "QfffffffB", time_us, state.wheel_rate_left_radps,
                                state.wheel_rate_right_radps, slip.residual_left,
                                slip.residual_right, slip.slip_left, slip.slip_right,
                                slip.confidence, slip_flags);

    // @LoggerMessage: TRJW
    // @Description: Trajectory mode nominal and final wheel-rate commands
    // @Field: TimeUS: Time since system startup
    // @Field: NomL: Nominal left wheel angular rate
    // @Field: NomR: Nominal right wheel angular rate
    // @Field: CmdL: Final left wheel angular rate
    // @Field: CmdR: Final right wheel angular rate
    // @Field: Flags: Wheel-rate output and slip compensation flags
    AP::logger().WriteStreaming("TRJW", "TimeUS,NomL,NomR,CmdL,CmdR,Flags",
                                "QffffB", time_us,
                                _control_output.wheel_rate_left_nominal_radps,
                                _control_output.wheel_rate_right_nominal_radps,
                                _control_output.wheel_rate_left_cmd_radps,
                                _control_output.wheel_rate_right_cmd_radps,
                                wheel_flags);
#endif
}

void ModeTrajectory::stop_with_warning(const char *reason)
{
    rover.trajectory.stop();
    rover.tidal_control.reset();
    _have_control_output = false;
    _terminal_complete = false;
    set_zero_output();
    if (!_failure_reported) {
        _failure_reported = true;
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "Trajectory stopped: %s", reason);
    }
}

void ModeTrajectory::update()
{
    if (!rover.arming.is_armed()) {
        stop_with_warning("vehicle disarmed");
        return;
    }
    if (SRV_Channels::get_emergency_stop()) {
        stop_with_warning("emergency stop");
        return;
    }
    if (_terminal_complete) {
        set_zero_output();
        return;
    }
    if (!rover.trajectory.running() && !rover.trajectory.finished()) {
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

    if (rover.trajectory.running()) {
        if (rover.trajectory.sample(now_us, _reference) != AR_Trajectory::Result::OK) {
            stop_with_warning("trajectory sampling failed");
            return;
        }
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

    if (rover.trajectory.finished() &&
        safe_sqrt(sq(_reference.x_m - state.x_m) + sq(_reference.y_m - state.y_m)) < 0.35f &&
        fabsf(state.speed_mps) < 0.15f) {
        rover.tidal_control.reset();
        _control_output = {};
        _have_control_output = false;
        _terminal_complete = true;
        set_zero_output();
        return;
    }

    const bool wheel_measurements_valid = state.wheel_left_valid && state.wheel_right_valid;
    const bool wheel_rate_output_available = wheel_measurements_valid &&
                                             g2.motors.have_skid_steering() &&
                                             g2.wheel_rate_control.enabled(0) &&
                                             g2.wheel_rate_control.enabled(1) &&
                                             is_positive(g2.wheel_rate_control.get_rate_max_rads());
    const bool use_ppc = g2.traj_ppc_enable != 0;
    const bool use_smo = g2.traj_smo_enable != 0 && wheel_measurements_valid;
    const bool use_slip_ppc = use_ppc && use_smo && g2.traj_slip_ppc_enable != 0;
    const bool use_wheel_compensation = use_smo && wheel_rate_output_available &&
                                        g2.traj_wcomp_enable != 0;
    rover.tidal_control.set_use_ppc(use_ppc);
    rover.tidal_control.set_use_slip_aware_ppc(use_slip_ppc);
    rover.tidal_control.set_use_residual_smo(use_smo);
    rover.tidal_control.set_use_wheel_compensation(use_wheel_compensation);
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

    write_log(state, progress);
    _have_control_output = true;
    if (!apply_output()) {
        stop_with_warning("wheel-rate output unavailable");
    }
}

#endif // MODE_TRAJECTORY_ENABLED
