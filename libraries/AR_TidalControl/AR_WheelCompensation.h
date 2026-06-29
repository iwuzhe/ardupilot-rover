#pragma once

#include "AR_TidalControl_config.h"
#include "AR_TidalControl_Types.h"

#if AR_TIDAL_CONTROL_ENABLED
class AR_WheelCompensation {
public:
    struct Config {
        float track_width_m = 1.2f;
        float wheel_radius_m = 0.127f;
        float slip_min = -1.20f;
        float slip_max = 1.20f;
        float slip_denominator_min = 0.25f;
        float slip_confidence_min = 0.1f;
        float wheel_accel_max_radps2 = 30.0f;
        float wheel_rate_max_radps = 12.0f;
        float dt_max_s = 0.1f;
    };

    AR_WheelCompensation();
    explicit AR_WheelCompensation(const Config &config);

    void reset();
    void set_wheel_rate_max(float wheel_rate_max_radps);
    bool update(float speed_cmd_mps, float yaw_rate_cmd_radps, float dt_s,
                const AR_SlipEstimate &slip, AR_TidalControlOutput &output);

private:
    Config _config;
    float _previous_left_radps = 0.0f;
    float _previous_right_radps = 0.0f;
};
#endif
