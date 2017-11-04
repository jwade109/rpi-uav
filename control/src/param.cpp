#include "param.h"

namespace uav
{

archive& operator << (archive& a, const param& p)
{
    return a << p.freq << p.mass << p.pid_gains;
}

archive& operator >> (archive& a, param& p)
{
    return a >> p.freq >> p.mass >> p.pid_gains;
}

} // namespace uav
