#include <angle.h>

namespace uav
{

angle::angle() : rotations(0), radians(0) { }

angle::angle(double rads, int rots) :
    rotations(rots + std::lround(rads/(2*M_PI))),
    radians(rads - (std::lround(rads/(2*M_PI))*2*M_PI)) { }

angle::angle(const angle& a) : radians(a.rad()), rotations(a.rot()) { }

double angle::rad() const { return radians; }

double angle::deg() const { return (180/M_PI) * radians; }

int angle::rot() const { return rotations; }

angle angle::operator - ()
{
    return *this * -1;
}

angle& angle::operator = (const angle& a)
{
    radians = a.rad();
    rotations = a.rot();
    return *this;
}

angle angle::operator + (const angle& a) const
{
    return angle(radians + a.rad(), rotations + a.rot());
}

angle angle::operator - (const angle& a) const
{
    return angle(radians - a.rad(), rotations - a.rot());
}

angle& angle::operator += (const angle& a)
{
    return (*this = *this + a);
}

angle& angle::operator -= (const angle& a)
{
    return (*this = *this - a);
}

bool angle::operator == (const angle& a) const
{
    return radians == a.rad() && rotations == a.rot();
}

bool angle::operator != (const angle& a) const
{
    return !(*this == a);
}

bool angle::operator > (const angle& a) const
{
    return rotations*2*M_PI + radians - a.rot()*2*M_PI - a.rad() > 0;
}

bool angle::operator < (const angle& a) const
{
    return rotations*2*M_PI + radians - a.rot()*2*M_PI - a.rad() < 0;
}

bool angle::operator >= (const angle& a) const
{
    return (*this == a) || (*this > a);
}

bool angle::operator <= (const angle& a) const
{
    return (*this == a) || (*this < a);
}

angle::operator double () const
{
    return radians + rotations*2*M_PI;;
}

std::ostream& operator << (std::ostream& os, const angle& a)
{
    return os << a.rot() << "x" << a.deg();
}

namespace angle_literals
{

angle operator "" _rad(unsigned long long radians)
{
    return angle(radians);
}

angle operator "" _deg(unsigned long long degrees)
{
    return angle(degrees * (M_PI/180));
}

angle operator "" _rad(long double radians)
{
    return angle(radians);
}

angle operator "" _deg(long double degrees)
{
    return angle(degrees * (M_PI/180));
}

} // namespace uav::angle_literals;

} // namespace uav
