#pragma once

#include "AR_TidalControl_config.h"
#include "AR_TidalControl_Types.h"

#include <AR_Trajectory/AR_Trajectory_Types.h>

#if AR_TIDAL_CONTROL_ENABLED
class AR_BacksteppingPPC {
public:
    struct Config {
        float gain_x = 1.2f;
        float gain_speed_p = 0.8f;
        float gain_speed_i = 0.1f;
        float speed_integral_max = 0.8f;
        float gain_y = 0.9f;
        float gain_yaw = 1.8f;
        float speed_max_mps = 2.0f;
        float yaw_rate_max_radps = 1.5707963267948966f;
        float speed_accel_max_mps2 = 0.127f * 30.0f;
        float yaw_accel_max_radps2 = 2.0f * 0.127f * 30.0f / 1.2f;
        float ppc_rho_initial_m = 0.60f;
        float ppc_rho_final_m = 0.08f;
        float ppc_progress_decay = 4.0f;
        float ppc_xi_max = 0.97f;
        float dt_max_s = 0.1f;
    };

    AR_BacksteppingPPC();
    explicit AR_BacksteppingPPC(const Config &config);

    void reset();
    bool update(const AR_TidalState &state, const AR_TrajectoryReference &reference,
                float trajectory_progress, bool use_ppc, AR_TidalControlOutput &output);

    const AR_BacksteppingPPCDebug &debug() const { return _debug; }

private:
    bool inputs_valid(const AR_TidalState &state, const AR_TrajectoryReference &reference,
                      float trajectory_progress) const;
    float rate_limit(float command, float previous, float rate_max, float dt_s) const;

    Config _config;
    AR_BacksteppingPPCDebug _debug {};
    float _speed_integral = 0.0f;
    float _previous_speed_cmd = 0.0f;
    float _previous_yaw_rate_cmd = 0.0f;
};
#endif
