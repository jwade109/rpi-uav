#include <angles.h>

namespace uav
{

angle::angle(double rad) : radians(atan2(sin(rad), cos(rad))) { }

angle::angle(const angle& a) : radians(a.rad()) { }

double angle::rad() const { return radians; }

double angle::deg() const { return (180/M_PI) * radians; }

angle& angle::operator = (const angle& a)
{
    radians = a.rad();
    return *this;
}

angle angle::operator + (const angle& a) const
{
    return angle(radians + a.rad());
}

angle angle::operator - (const angle& a) const
{
    return angle(radians - a.rad());
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
    return radians == a.rad();
}

bool angle::operator != (const angle& a) const
{
    return !(*this == a);
}

bool angle::operator > (const angle& a) const
{
    return (*this - a).rad() > 0;
}

bool angle::operator < (const angle& a) const
{
    return (*this - a).rad() < 0;
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
    return radians;
}

std::ostream& operator << (std::ostream& os, const angle& a)
{
    return os << a.deg();
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
