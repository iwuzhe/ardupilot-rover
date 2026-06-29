#pragma once

#include "AR_BacksteppingPPC.h"
#include "AR_ResidualSMO.h"
#include "AR_TidalControl_Types.h"
#include "AR_WheelCompensation.h"

#include <AR_Trajectory/AR_Trajectory_Types.h>

#if AR_TIDAL_CONTROL_ENABLED
class AR_TidalControl {
public:
    void reset();
    bool update(const AR_TidalState &state, const AR_TrajectoryReference &reference,
                float trajectory_progress, AR_TidalControlOutput &output);

    void set_use_ppc(bool enable) { _use_ppc = enable; }
    void set_use_residual_smo(bool enable) { _use_residual_smo = enable; }
    void set_use_wheel_compensation(bool enable) { _use_wheel_compensation = enable; }

    bool use_ppc() const { return _use_ppc; }
    bool use_residual_smo() const { return _use_residual_smo; }
    bool use_wheel_compensation() const { return _use_wheel_compensation; }

    const AR_BacksteppingPPCDebug &backstepping_debug() const { return _backstepping.debug(); }

private:
    AR_ResidualSMO _residual_smo;
    AR_BacksteppingPPC _backstepping;
    AR_WheelCompensation _wheel_compensation;
    bool _use_ppc = true;
    bool _use_residual_smo = false;
    bool _use_wheel_compensation = false;
};
#endif
