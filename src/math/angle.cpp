#include "angle.h"

#include <iomanip>
#include <sstream>

namespace uav
{

angle angle::radians(double rads)
{
    return angle(radians, 0);
}

angle angle::rotations(double rots)
{
    return angle(0, rots);
}

angle angle::degrees(double degs)
{
    return angle(degs/
}

static angle minutes(double mins);
static angle seconds(double secs);
static angle milliseconds(double ms);

angle();
angle(double rads, int rots = 0);
angle(uint8_t d, uint8_t m, uint8_t s, double ms, bool dir)
angle(const angle& a);

angle angle::from_degrees(double degs) { return angle(degs*(M_PI/180), 0); }

angle angle::from_radians(double rads) { return angle(rads, 0); }

angle angle::from_rotations(int rots) { return angle(0, rots); }

angle::angle() : angle(0, 0) { }

angle::angle(double rads, int rots) :
    rotations(rots + std::lround(rads/(2*M_PI))),
    radians(rads - (std::lround(rads/(2*M_PI))*2*M_PI)) { }

angle::angle(const angle& a) : rotations(a.rot()), radians(a.rad()) { }

double angle::rad() const { return radians; }

double angle::deg() const { return (180/M_PI) * radians; }

int angle::rot() const { return rotations; }

angle angle::operator - ()
{
    return angle(-radians, -rotations);
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

angle& angle::operator = (double rads)
{
    return *this = angle(rads);
}

angle angle::operator * (double scalar) const
{
    return angle((radians + 2*M_PI * rotations) * scalar);
}

angle angle::operator / (double divisor) const
{
    return *this * (1/divisor);
}

angle& angle::operator *= (double scalar)
{
    return (*this = *this * scalar);
}

angle& angle::operator /= (double divisor)
{
    return (*this = *this / divisor);
}

double angle::operator / (const angle& a) const
{
    return (radians + 2*M_PI*rotations) / (a.rad() + 2*M_PI*a.rot());
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

angle target_azimuth(angle current, angle desired)
{
    angle diff((desired - current).rad());
    return current + diff;
}

std::ostream& operator << (std::ostream& os, const angle& a)
{
    std::stringstream ss;
    ss << std::fixed << std::setprecision(3) << a.rot() << "*" << a.deg();
    return os << ss.str();
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
