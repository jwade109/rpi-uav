#include "angle.h"

#include <iomanip>
#include <sstream>

namespace uav
{

angle::angle(int64_t us, bool) : _micros(us) { }

angle::angle() : _micros(0) { }

angle::angle(const angle& a) : _micros(a.micros()) { }

angle::angle(double rads) : _micros(rads * us_rad) { }

int64_t angle::micros() const
{
    return _micros;
}

int64_t& angle::micros()
{
    return _micros;
}

double angle::mil() const
{
    return (double) _micros / us_ms;
}

double angle::sec() const
{
    return (double) _micros / us_sec;
}

double angle::min() const
{
    return (double) _micros / us_min;
}

double angle::deg() const
{
    return (double) _micros / us_deg;
}

double angle::rad() const
{
    return (double) _micros / us_rad;
}

double angle::rev() const
{
    return (double) _micros / us_rev;
}

angle angle::operator - ()
{
    return angle(-_micros, true);
}

angle::operator double () const
{
    return (double) _micros / us_rad;
}

angle& angle::operator = (const angle& a)
{
    _micros = a.micros();
    return *this;
}

angle angle::operator + (const angle& a) const
{
    return angle(_micros + a.micros(), true);
}

angle angle::operator - (const angle& a) const
{
    return angle(_micros - a.micros(), true);
}

angle& angle::operator += (const angle& a)
{
    return (*this = *this + a);
}

angle& angle::operator -= (const angle& a)
{
    return (*this = *this - a);
}

angle angle::operator / (double divisor) const
{
    return *this * (1/divisor);
}

angle& angle::operator /= (double divisor)
{
    return (*this = *this / divisor);
}

double angle::operator / (const angle& a) const
{
    return (double) _micros / a.micros();
}

bool angle::operator == (const angle& a) const
{
    return _micros == a.micros();
}

bool angle::operator != (const angle& a) const
{
    return !(*this == a);
}

bool angle::operator > (const angle& a) const
{
    return _micros > a.micros();
}

bool angle::operator < (const angle& a) const
{
    return _micros < a.micros();
}

bool angle::operator >= (const angle& a) const
{
    return !(*this < a);
}

bool angle::operator <= (const angle& a) const
{
    return !(*this > a);
}

std::ostream& operator << (std::ostream& os, const angle& a)
{
    std::stringstream ss;
    ss << std::fixed << std::setprecision(6) << a.deg();
    return os << ss.str();
}

namespace angle_literals
{

angle operator "" _rad(unsigned long long radians)
{
    return angle::radians(radians);
}

angle operator "" _rad(long double radians)
{
    return angle::radians(radians);
}

angle operator "" _deg(unsigned long long degrees)
{
    return angle::degrees(degrees);
}

angle operator "" _deg(long double degrees)
{
    return angle::degrees(degrees);
}

angle operator "" _rev(unsigned long long revolutions)
{
    return angle::revolutions(revolutions);
}

angle operator "" _rev(long double revolutions)
{
    return angle::revolutions(revolutions);
}

angle operator "" _min(unsigned long long minutes)
{
    return angle::minutes(minutes);
}

angle operator "" _min(long double minutes)
{
    return angle::minutes(minutes);
}

angle operator "" _sec(unsigned long long seconds)
{
    return angle::seconds(seconds);
}

angle operator "" _sec(long double seconds)
{
    return angle::seconds(seconds);
}

angle operator "" _ms(unsigned long long ms)
{
    return angle::milliseconds(ms);
}

angle operator "" _ms(long double ms)
{
    return angle::milliseconds(ms);
}

angle operator "" _us(unsigned long long us)
{
    return angle::microseconds(us);
}

angle operator "" _us(long double us)
{
    return angle::microseconds(us);
}

} // namespace uav::angle_literals;

} // namespace uav
