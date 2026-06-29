#include "AR_BacksteppingPPC.h"

#include <AP_Math/AP_Math.h>

#if AR_TIDAL_CONTROL_ENABLED
AR_BacksteppingPPC::AR_BacksteppingPPC()
{
}

AR_BacksteppingPPC::AR_BacksteppingPPC(const Config &config) :
    _config(config)
{
}

void AR_BacksteppingPPC::reset()
{
    _debug = {};
    _speed_integral = 0.0f;
    _previous_speed_cmd = 0.0f;
    _previous_yaw_rate_cmd = 0.0f;
}

bool AR_BacksteppingPPC::inputs_valid(const AR_TidalState &state,
                                      const AR_TrajectoryReference &reference,
                                      float trajectory_progress) const
{
    return state.position_valid && state.velocity_valid &&
           is_positive(state.dt_s) && state.dt_s <= _config.dt_max_s &&
           isfinite(state.x_m) && isfinite(state.y_m) &&
           isfinite(state.yaw_rad) && isfinite(state.speed_mps) &&
           isfinite(state.yaw_rate_radps) &&
           isfinite(reference.time_s) && isfinite(reference.x_m) &&
           isfinite(reference.y_m) && isfinite(reference.yaw_rad) &&
           isfinite(reference.speed_mps) && isfinite(reference.yaw_rate_radps) &&
           isfinite(trajectory_progress);
}

float AR_BacksteppingPPC::rate_limit(float command, float previous, float rate_max, float dt_s) const
{
    const float maximum_change = rate_max * dt_s;
    return previous + constrain_float(command - previous, -maximum_change, maximum_change);
}

bool AR_BacksteppingPPC::update(const AR_TidalState &state,
                                const AR_TrajectoryReference &reference,
                                float trajectory_progress, const AR_SlipEstimate &slip,
                                bool use_ppc, bool use_slip_aware_ppc,
                                AR_TidalControlOutput &output)
{
    output = {};
    output.stop_required = true;
    _debug = {};

    if (!inputs_valid(state, reference, trajectory_progress)) {
        return false;
    }

    const float progress = constrain_float(trajectory_progress, 0.0f, 1.0f);
    const float delta_x = reference.x_m - state.x_m;
    const float delta_y = reference.y_m - state.y_m;
    const float cos_yaw = cosf(state.yaw_rad);
    const float sin_yaw = sinf(state.yaw_rad);
    const float error_x = cos_yaw * delta_x + sin_yaw * delta_y;
    const float error_y = -sin_yaw * delta_x + cos_yaw * delta_y;
    const float error_y_control = sinf(reference.yaw_rad) * (state.x_m - reference.x_m) -
                                  cosf(reference.yaw_rad) * (state.y_m - reference.y_m);
    const float error_yaw = wrap_PI(reference.yaw_rad - state.yaw_rad);
    const float error_speed = reference.speed_mps - state.speed_mps;

    _speed_integral = constrain_float(_speed_integral + error_speed * state.dt_s,
                                      -_config.speed_integral_max,
                                      _config.speed_integral_max);

    float transformed_error_y = error_y_control;
    float rho_y = 0.0f;
    float rho_base = 0.0f;
    float xi_y = 0.0f;
    float slip_factor = 0.0f;
    bool slip_scheduled = false;
    bool ppc_fallback = false;
    bool ppc_violation = false;

    if (use_ppc) {
        rho_base = _config.ppc_rho_final_m +
                   (_config.ppc_rho_initial_m - _config.ppc_rho_final_m) *
                   expf(-_config.ppc_progress_decay * progress);
        rho_y = rho_base;
        if (use_slip_aware_ppc && slip.valid &&
            isfinite(slip.slip_left) && isfinite(slip.slip_right)) {
            const float slip_left = constrain_float(slip.slip_left,
                                                    _config.ppc_slip_min,
                                                    _config.ppc_slip_max);
            const float slip_right = constrain_float(slip.slip_right,
                                                     _config.ppc_slip_min,
                                                     _config.ppc_slip_max);
            const float slip_difference = fabsf(slip_right - slip_left);
            const float schedule_denominator = MAX(_config.ppc_slip_full -
                                                   _config.ppc_slip_deadzone,
                                                   1.0e-6f);
            slip_factor = 0.5f *
                          (1.0f + tanhf((slip_difference - _config.ppc_slip_deadzone) /
                                        schedule_denominator));
            rho_y = MAX(rho_base + _config.ppc_rho_relax_m * slip_factor,
                        _config.ppc_rho_final_m);
            slip_scheduled = true;
        }
        if (!is_positive(rho_y) || !isfinite(rho_y)) {
            ppc_fallback = true;
            ppc_violation = true;
        } else {
            const float xi_unconstrained = error_y_control / rho_y;
            ppc_violation = fabsf(xi_unconstrained) >= _config.ppc_xi_max;
            if (fabsf(xi_unconstrained) >= 1.0f || !isfinite(xi_unconstrained)) {
                ppc_fallback = true;
            } else {
                xi_y = constrain_float(xi_unconstrained,
                                       -_config.ppc_xi_max,
                                       _config.ppc_xi_max);
                transformed_error_y = 0.5f * logf((1.0f + xi_y) / (1.0f - xi_y));
                if (!isfinite(transformed_error_y)) {
                    transformed_error_y = error_y_control;
                    ppc_fallback = true;
                    ppc_violation = true;
                }
            }
        }
    }

    if (ppc_fallback) {
        transformed_error_y = error_y_control;
        xi_y = 0.0f;
    }

    float speed_cmd = reference.speed_mps * cosf(error_yaw) +
                      _config.gain_x * error_x +
                      _config.gain_speed_p * error_speed +
                      _config.gain_speed_i * _speed_integral;
    float yaw_rate_cmd = reference.yaw_rate_radps +
                         _config.gain_y * reference.speed_mps * transformed_error_y +
                         _config.gain_yaw * error_yaw;

    if (!isfinite(speed_cmd) || !isfinite(yaw_rate_cmd)) {
        return false;
    }

    speed_cmd = constrain_float(speed_cmd, -_config.speed_max_mps, _config.speed_max_mps);
    yaw_rate_cmd = constrain_float(yaw_rate_cmd,
                                   -_config.yaw_rate_max_radps,
                                   _config.yaw_rate_max_radps);
    speed_cmd = rate_limit(speed_cmd, _previous_speed_cmd,
                           _config.speed_accel_max_mps2, state.dt_s);
    yaw_rate_cmd = rate_limit(yaw_rate_cmd, _previous_yaw_rate_cmd,
                              _config.yaw_accel_max_radps2, state.dt_s);

    if (!isfinite(speed_cmd) || !isfinite(yaw_rate_cmd)) {
        return false;
    }

    _previous_speed_cmd = speed_cmd;
    _previous_yaw_rate_cmd = yaw_rate_cmd;

    output.speed_cmd_mps = speed_cmd;
    output.yaw_rate_cmd_radps = yaw_rate_cmd;
    output.valid = true;
    output.stop_required = false;
    output.ppc_violation = ppc_violation;

    _debug.error_x_m = error_x;
    _debug.error_y_m = error_y;
    _debug.error_y_control_m = error_y_control;
    _debug.error_y_transformed = transformed_error_y;
    _debug.error_yaw_rad = error_yaw;
    _debug.error_speed_mps = error_speed;
    _debug.ppc_rho_m = rho_y;
    _debug.ppc_rho_base_m = rho_base;
    _debug.ppc_xi = xi_y;
    _debug.ppc_slip_factor = slip_factor;
    _debug.trajectory_progress = progress;
    _debug.ppc_active = use_ppc && !ppc_fallback;
    _debug.ppc_fallback = ppc_fallback;
    _debug.ppc_slip_scheduled = slip_scheduled;
    return true;
}
#endif
