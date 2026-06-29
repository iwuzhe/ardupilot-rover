#pragma once

#include "AR_TidalControl_config.h"
#include "AR_TidalControl_Types.h"

#if AR_TIDAL_CONTROL_ENABLED
class AR_ResidualSMO {
public:
    void reset();
    bool update(const AR_TidalState &state, AR_SlipEstimate &estimate);
};
#endif
