#pragma once

#include <stdint.h>

struct AR_TrajectoryPoint {
    float time_s;
    float x_m;
    float y_m;
    float yaw_rad;
    float speed_mps;
    float yaw_rate_radps;
};

struct AR_TrajectoryReference : AR_TrajectoryPoint {
    uint16_t segment_index;
};

