#ifndef FUNCTIONS_H
#define FUNCTIONS_H

#include "angle.h"
#include "vector.h"
#include "coordinate.h"

namespace uav
{

angle target_azimuth(const angle& current, const angle& desired);

imu::Vector<3> displacement(const coordinate& a, const coordinate& b);

} // namespace uav

#endif
