#include "AR_WheelCompensation.h"

#if AR_TIDAL_CONTROL_ENABLED
void AR_WheelCompensation::reset()
{
}

bool AR_WheelCompensation::update(float, float, float,
                                  const AR_SlipEstimate &, AR_TidalControlOutput &output)
{
    // Wheel compensation is implemented in stage 12. Do not expose wheel
    // commands until the native wheel-rate control path has been validated.
    output.wheel_rate_left_cmd_radps = 0.0f;
    output.wheel_rate_right_cmd_radps = 0.0f;
    output.wheel_rate_output = false;
    return false;
}
#endif
