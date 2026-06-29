#pragma once

#include "AR_TidalControl_config.h"
#include "AR_TidalControl_Types.h"

#if AR_TIDAL_CONTROL_ENABLED
class AR_WheelCompensation {
public:
    void reset();
    bool update(float speed_cmd_mps, float yaw_rate_cmd_radps, float dt_s,
                const AR_SlipEstimate &slip, AR_TidalControlOutput &output);
};
#endif
