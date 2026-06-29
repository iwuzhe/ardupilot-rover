#include "AR_TidalControl.h"

#if AR_TIDAL_CONTROL_ENABLED
void AR_TidalControl::reset()
{
    _residual_smo.reset();
    _backstepping.reset();
    _wheel_compensation.reset();
}

bool AR_TidalControl::update(const AR_TidalState &state,
                             const AR_TrajectoryReference &reference,
                             float trajectory_progress,
                             AR_TidalControlOutput &output)
{
    output = {};
    output.stop_required = true;

    AR_SlipEstimate slip {};
    if (_use_residual_smo && !_residual_smo.update(state, slip)) {
        return false;
    }

    if (!_backstepping.update(state, reference, trajectory_progress, _use_ppc, output)) {
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
