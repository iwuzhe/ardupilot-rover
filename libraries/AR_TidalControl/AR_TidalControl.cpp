#include "AR_TidalControl.h"

#if AR_TIDAL_CONTROL_ENABLED
void AR_TidalControl::reset()
{
    _residual_smo.reset();
    _backstepping.reset();
    _wheel_compensation.reset();
    _slip_estimate = {};
}

void AR_TidalControl::set_use_residual_smo(bool enable)
{
    if (_use_residual_smo != enable) {
        _residual_smo.reset();
        _slip_estimate = {};
        _use_residual_smo = enable;
    }
}

void AR_TidalControl::set_use_wheel_compensation(bool enable)
{
    if (_use_wheel_compensation != enable) {
        _wheel_compensation.reset();
        _use_wheel_compensation = enable;
    }
}

void AR_TidalControl::set_wheel_rate_limit(float wheel_rate_max_radps)
{
    _wheel_compensation.set_wheel_rate_max(wheel_rate_max_radps);
}

bool AR_TidalControl::update(const AR_TidalState &state,
                             const AR_TrajectoryReference &reference,
                             float trajectory_progress,
                             AR_TidalControlOutput &output)
{
    output = {};
    output.stop_required = true;

    if (_use_residual_smo && !_residual_smo.update(state, _slip_estimate)) {
        return false;
    }
    const AR_SlipEstimate slip = _use_residual_smo ? _slip_estimate : AR_SlipEstimate {};

    if (!_backstepping.update(state, reference, trajectory_progress, slip,
                              _use_ppc, _use_slip_aware_ppc, output)) {
        return false;
    }

    if (_use_wheel_compensation &&
        !_wheel_compensation.update(output.speed_cmd_mps, output.yaw_rate_cmd_radps,
                                    state.dt_s, slip, output)) {
        output = {};
        output.stop_required = true;
        return false;
    }

    return output.valid && !output.stop_required;
}
#endif
