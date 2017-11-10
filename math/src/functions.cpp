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

} // namespace uav
