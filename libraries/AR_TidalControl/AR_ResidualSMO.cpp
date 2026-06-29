#include "AR_ResidualSMO.h"

#if AR_TIDAL_CONTROL_ENABLED
void AR_ResidualSMO::reset()
{
}

bool AR_ResidualSMO::update(const AR_TidalState &, AR_SlipEstimate &estimate)
{
    // Residual-SMO is implemented in stage 11. Keep an explicit, safe
    // placeholder so enabling it early cannot produce a usable estimate.
    estimate = {};
    return false;
}
#endif
