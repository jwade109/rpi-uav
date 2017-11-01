#include "functions.h"

namespace uav
{

double altitude(double p_atm)
{
    return 44330 * (1.0 - pow(p_atm, 0.1903));
}

angle target_azimuth(const angle& current, const angle& target)
{
    return current;
}

imu::Vector<3> displacement(const coordinate& a, const coordinate& b)
{
    auto d = b - a;
    imu::Vector<3> v;
    v.x() = d.longitude().mil() * 0.03092; // 3.092 cm/mil
    v.y() = d.longitude().mil() * 0.03092;
    v.z() = d.altitude();
    return v;
}

} // namespace uav
