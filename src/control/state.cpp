#include "state.h"

namespace uav
{

archive& operator << (archive& a, const angle& b)
{
    return a << b.micros();
}

archive& operator >> (archive& a, angle& b)
{
    return a >> b.micros();
}

archive& operator << (archive& a, const coordinate& c)
{
    return a << c.latitude() << c.longitude() << c.altitude();
}

archive& operator >> (archive& a, coordinate& c)
{
    return a >> c.latitude() >> c.longitude() >> c.altitude();
}

archive& operator << (archive& a, const state& s)
{
    a << s.time << s.position;
    for (angle e : s.attitude) a << e;
    for (angle e : s.targets) a << e;
    return a << s.motors << s.error << s.status;
}

archive& operator >> (archive& a, state& s)
{
    a >> s.time >> s.position;
    for (angle e : s.attitude) a >> e;
    for (angle e : s.targets) a >> e;
    return a >> s.motors >> s.error >> s.status;
}

} // namespace uav
