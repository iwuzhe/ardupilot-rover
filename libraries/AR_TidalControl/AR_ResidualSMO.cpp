#include "AR_ResidualSMO.h"

#include <AP_Math/AP_Math.h>

#if AR_TIDAL_CONTROL_ENABLED
AR_ResidualSMO::AR_ResidualSMO()
{
}

AR_ResidualSMO::AR_ResidualSMO(const Config &config) :
    _config(config)
{
}

void AR_ResidualSMO::reset()
{
    _estimate = {};
    _slip_left = 0.0f;
    _slip_right = 0.0f;
    _track_speed_left_filtered = 0.0f;
    _track_speed_right_filtered = 0.0f;
    _previous_speed_mps = 0.0f;
    _wheel_left_time_ms = 0;
    _wheel_right_time_ms = 0;
    _measurement_time_ms = 0;
    _initialised = false;
}

void AR_ResidualSMO::initialise(const AR_TidalState &state)
{
    _track_speed_left_filtered = _config.wheel_radius_m * state.wheel_rate_left_radps;
    _track_speed_right_filtered = _config.wheel_radius_m * state.wheel_rate_right_radps;
    _previous_speed_mps = state.speed_mps;
    _wheel_left_time_ms = state.wheel_left_time_ms;
    _wheel_right_time_ms = state.wheel_right_time_ms;
    _measurement_time_ms = MAX(_wheel_left_time_ms, _wheel_right_time_ms);
    _estimate = {};
    _estimate.measurement_time_ms = _measurement_time_ms;
    _initialised = true;
}

bool AR_ResidualSMO::update(const AR_TidalState &state, AR_SlipEstimate &estimate)
{
    _estimate.new_data = false;

    if (!is_positive(state.dt_s) || state.dt_s > _config.dt_max_s ||
        !isfinite(state.speed_mps) || !isfinite(state.yaw_rate_radps)) {
        _estimate.valid = false;
        estimate = _estimate;
        return false;
    }

    if (!state.wheel_left_valid || !state.wheel_right_valid ||
        !isfinite(state.wheel_rate_left_radps) ||
        !isfinite(state.wheel_rate_right_radps)) {
        _estimate.valid = false;
        estimate = _estimate;
        return true;
    }

    if (!_initialised) {
        initialise(state);
        estimate = _estimate;
        return true;
    }

    if (state.wheel_left_time_ms == _wheel_left_time_ms ||
        state.wheel_right_time_ms == _wheel_right_time_ms) {
        estimate = _estimate;
        return true;
    }

    const uint32_t measurement_time_ms = MAX(state.wheel_left_time_ms,
                                             state.wheel_right_time_ms);
    const uint32_t dt_ms = measurement_time_ms - _measurement_time_ms;
    const float dt_s = dt_ms * 0.001f;
    if (dt_ms == 0 || !is_positive(dt_s) || dt_s > _config.dt_max_s) {
        reset();
        initialise(state);
        estimate = _estimate;
        return true;
    }

    const float track_speed_left_raw = _config.wheel_radius_m * state.wheel_rate_left_radps;
    const float track_speed_right_raw = _config.wheel_radius_m * state.wheel_rate_right_radps;
    const float track_filter_alpha = constrain_float(
        dt_s / (_config.track_filter_tau_s + dt_s), 0.0f, 1.0f);
    _track_speed_left_filtered += track_filter_alpha *
                                  (track_speed_left_raw - _track_speed_left_filtered);
    _track_speed_right_filtered += track_filter_alpha *
                                   (track_speed_right_raw - _track_speed_right_filtered);

    const float acceleration_mps2 = (state.speed_mps - _previous_speed_mps) / dt_s;
    const float turn_intensity = MIN(fabsf(state.yaw_rate_radps) /
                                     MAX(_config.yaw_rate_reference_radps, 1.0e-6f), 1.0f);
    const float acceleration_intensity = MIN(fabsf(acceleration_mps2) /
                                             MAX(_config.acceleration_reference_mps2, 1.0e-6f), 1.0f);
    const float gamma_effective = MIN(
        _config.gamma_base * (1.0f + 0.8f * turn_intensity + 0.6f * acceleration_intensity),
        _config.gamma_max);

    // ArduPilot uses NED yaw-rate, positive clockwise, so the left track is
    // faster during a positive turn.
    const float body_speed_left = state.speed_mps +
                                  0.5f * _config.track_width_m * state.yaw_rate_radps;
    const float body_speed_right = state.speed_mps -
                                   0.5f * _config.track_width_m * state.yaw_rate_radps;
    const float residual_left = (1.0f - _slip_left) * _track_speed_left_filtered -
                                body_speed_left;
    const float residual_right = (1.0f - _slip_right) * _track_speed_right_filtered -
                                 body_speed_right;
    const float sigma_left = constrain_float(
        residual_left / MAX(_config.boundary_layer_mps, 1.0e-6f), -1.0f, 1.0f);
    const float sigma_right = constrain_float(
        residual_right / MAX(_config.boundary_layer_mps, 1.0e-6f), -1.0f, 1.0f);
    const float observable_left = MIN(
        fabsf(_track_speed_left_filtered) / MAX(_config.observable_speed_mps, 1.0e-6f), 1.0f);
    const float observable_right = MIN(
        fabsf(_track_speed_right_filtered) / MAX(_config.observable_speed_mps, 1.0e-6f), 1.0f);

    _slip_left += dt_s *
                  (gamma_effective * observable_left * _track_speed_left_filtered * sigma_left -
                   _config.low_speed_leak_rate * (1.0f - observable_left) * _slip_left);
    _slip_right += dt_s *
                   (gamma_effective * observable_right * _track_speed_right_filtered * sigma_right -
                    _config.low_speed_leak_rate * (1.0f - observable_right) * _slip_right);
    _slip_left = constrain_float(_slip_left, _config.slip_min, _config.slip_max);
    _slip_right = constrain_float(_slip_right, _config.slip_min, _config.slip_max);

    const float output_filter_alpha = constrain_float(
        dt_s / (_config.output_filter_tau_s + dt_s), 0.0f, 1.0f);
    _estimate.slip_left += output_filter_alpha * (_slip_left - _estimate.slip_left);
    _estimate.slip_right += output_filter_alpha * (_slip_right - _estimate.slip_right);
    _estimate.residual_left = residual_left;
    _estimate.residual_right = residual_right;
    _estimate.confidence = 0.5f * (observable_left + observable_right);
    _estimate.measurement_time_ms = measurement_time_ms;
    _estimate.valid = _estimate.confidence >= _config.confidence_min &&
                      isfinite(_estimate.slip_left) && isfinite(_estimate.slip_right);
    _estimate.new_data = true;

    _previous_speed_mps = state.speed_mps;
    _wheel_left_time_ms = state.wheel_left_time_ms;
    _wheel_right_time_ms = state.wheel_right_time_ms;
    _measurement_time_ms = measurement_time_ms;
    estimate = _estimate;
    return true;
}
#endif
