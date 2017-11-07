#ifndef FUNCTIONS_H
#define FUNCTIONS_H

#include "angle.h"

namespace uav
{

double altitude(double p_atm);

angle target_azimuth(const angle& current, const angle& desired);

} // namespace uav

#endif
