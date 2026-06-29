#pragma once

#include "AR_TidalControl_config.h"
#include "AR_TidalControl_Types.h"

#if AR_TIDAL_CONTROL_ENABLED
class AR_ResidualSMO {
public:
    struct Config {
        float track_width_m = 1.2f;
        float wheel_radius_m = 0.127f;
        float track_filter_tau_s = 0.010f;
        float gamma_base = 3.5f;
        float gamma_max = 5.0f;
        float yaw_rate_reference_radps = 0.40f;
        float acceleration_reference_mps2 = 1.0f;
        float boundary_layer_mps = 0.10f;
        float observable_speed_mps = 0.25f;
        float low_speed_leak_rate = 0.35f;
        float slip_min = -1.20f;
        float slip_max = 1.20f;
        float output_filter_tau_s = 0.04f;
        float confidence_min = 0.1f;
        float dt_max_s = 0.1f;
    };

    AR_ResidualSMO();
    explicit AR_ResidualSMO(const Config &config);

    void reset();
    bool update(const AR_TidalState &state, AR_SlipEstimate &estimate);

private:
    void initialise(const AR_TidalState &state);

    Config _config;
    AR_SlipEstimate _estimate {};
    float _slip_left = 0.0f;
    float _slip_right = 0.0f;
    float _track_speed_left_filtered = 0.0f;
    float _track_speed_right_filtered = 0.0f;
    float _previous_speed_mps = 0.0f;
    uint32_t _wheel_left_time_ms = 0;
    uint32_t _wheel_right_time_ms = 0;
    uint32_t _measurement_time_ms = 0;
    bool _initialised = false;
};
#endif
