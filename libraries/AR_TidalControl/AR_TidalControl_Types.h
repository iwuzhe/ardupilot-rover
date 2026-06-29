#pragma once

#include <stdint.h>

struct AR_TidalState {
    float x_m;
    float y_m;
    float yaw_rad;
    float speed_mps;
    float yaw_rate_radps;
    float wheel_rate_left_radps;
    float wheel_rate_right_radps;
    float dt_s;
    uint32_t wheel_left_time_ms;
    uint32_t wheel_right_time_ms;
    bool position_valid;
    bool velocity_valid;
    bool wheel_left_valid;
    bool wheel_right_valid;
};

struct AR_SlipEstimate {
    float slip_left;
    float slip_right;
    float residual_left;
    float residual_right;
    float confidence;
    bool valid;
};

struct AR_TidalControlOutput {
    float speed_cmd_mps;
    float yaw_rate_cmd_radps;
    float wheel_rate_left_cmd_radps;
    float wheel_rate_right_cmd_radps;
    bool valid;
    bool stop_required;
    bool ppc_violation;
    bool wheel_rate_output;
};

struct AR_BacksteppingPPCDebug {
    float error_x_m;
    float error_y_m;
    float error_y_control_m;
    float error_y_transformed;
    float error_yaw_rad;
    float error_speed_mps;
    float ppc_rho_m;
    float ppc_xi;
    float trajectory_progress;
    bool ppc_active;
    bool ppc_fallback;
};
