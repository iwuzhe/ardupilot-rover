#include "AR_WheelCompensation.h"

#include <AP_Math/AP_Math.h>

#if AR_TIDAL_CONTROL_ENABLED
AR_WheelCompensation::AR_WheelCompensation()
{
}

AR_WheelCompensation::AR_WheelCompensation(const Config &config) :
    _config(config)
{
}

void AR_WheelCompensation::reset()
{
    _previous_left_radps = 0.0f;
    _previous_right_radps = 0.0f;
}

void AR_WheelCompensation::set_wheel_rate_max(float wheel_rate_max_radps)
{
    if (is_positive(wheel_rate_max_radps) && isfinite(wheel_rate_max_radps)) {
        _config.wheel_rate_max_radps = wheel_rate_max_radps;
    }
}

bool AR_WheelCompensation::update(float speed_cmd_mps, float yaw_rate_cmd_radps, float dt_s,
                                  const AR_SlipEstimate &slip, AR_TidalControlOutput &output)
{
    if (!isfinite(speed_cmd_mps) || !isfinite(yaw_rate_cmd_radps) ||
        !is_positive(dt_s) || dt_s > _config.dt_max_s ||
        !is_positive(_config.wheel_radius_m) ||
        !is_positive(_config.wheel_rate_max_radps)) {
        return false;
    }

    // ArduPilot NED yaw-rate is positive clockwise. A positive turn therefore
    // requires the left track to move faster than the right track.
    const float track_speed_left = speed_cmd_mps +
                                   0.5f * _config.track_width_m * yaw_rate_cmd_radps;
    const float track_speed_right = speed_cmd_mps -
                                    0.5f * _config.track_width_m * yaw_rate_cmd_radps;
    const float nominal_left_radps = track_speed_left / _config.wheel_radius_m;
    const float nominal_right_radps = track_speed_right / _config.wheel_radius_m;

    float compensated_left_radps = nominal_left_radps;
    float compensated_right_radps = nominal_right_radps;
    const bool apply_slip_compensation = slip.valid &&
                                         slip.confidence >= _config.slip_confidence_min &&
                                         isfinite(slip.slip_left) && isfinite(slip.slip_right);
    if (apply_slip_compensation) {
        const float slip_left = constrain_float(slip.slip_left,
                                                _config.slip_min,
                                                _config.slip_max);
        const float slip_right = constrain_float(slip.slip_right,
                                                 _config.slip_min,
                                                 _config.slip_max);
        const float denominator_left = MAX(1.0f - slip_left,
                                           _config.slip_denominator_min);
        const float denominator_right = MAX(1.0f - slip_right,
                                            _config.slip_denominator_min);
        compensated_left_radps /= denominator_left;
        compensated_right_radps /= denominator_right;
    }

    if (!isfinite(compensated_left_radps) || !isfinite(compensated_right_radps)) {
        return false;
    }

    const float largest_rate = MAX(fabsf(compensated_left_radps),
                                   fabsf(compensated_right_radps));
    if (largest_rate > _config.wheel_rate_max_radps) {
        const float scale = _config.wheel_rate_max_radps / largest_rate;
        compensated_left_radps *= scale;
        compensated_right_radps *= scale;
    }

    const float maximum_change = _config.wheel_accel_max_radps2 * dt_s;
    const float final_left_radps = _previous_left_radps +
                                   constrain_float(compensated_left_radps - _previous_left_radps,
                                                   -maximum_change, maximum_change);
    const float final_right_radps = _previous_right_radps +
                                    constrain_float(compensated_right_radps - _previous_right_radps,
                                                    -maximum_change, maximum_change);
    if (!isfinite(final_left_radps) || !isfinite(final_right_radps)) {
        return false;
    }

    _previous_left_radps = final_left_radps;
    _previous_right_radps = final_right_radps;
    output.wheel_rate_left_nominal_radps = nominal_left_radps;
    output.wheel_rate_right_nominal_radps = nominal_right_radps;
    output.wheel_rate_left_cmd_radps = final_left_radps;
    output.wheel_rate_right_cmd_radps = final_right_radps;
    output.wheel_rate_output = true;
    output.slip_compensation_applied = apply_slip_compensation;
    return true;
}
#endif
