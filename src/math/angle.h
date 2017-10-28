#ifndef ANGLE_H
#define ANGLE_H

#include <iostream>
#include <cmath>

namespace uav
{

class angle
{
    public:

    template <typename T> static angle revolutions(T revs);
    template <typename T> static angle radians(T rads);
    template <typename T> static angle degrees(T degs);
    template <typename T> static angle minutes(T mins);
    template <typename T> static angle seconds(T secs);
    template <typename T> static angle milliseconds(T ms);
    template <typename T> static angle microseconds(T us); 

    angle();
    angle(const angle& a);
    angle(double rads);

    int64_t micros() const;
    double mil() const;
    double sec() const;
    double min() const;
    double deg() const;
    double rad() const;
    double rev() const;

    operator double() const;
    angle& operator = (const angle& a);

    angle operator - ();
    angle operator + (const angle& a) const;
    angle operator - (const angle& a) const;
    angle& operator += (const angle& a);
    angle& operator -= (const angle& a);

    template <typename T> angle operator * (T scalar) const;
    template <typename T> angle& operator *= (T scalar);
    angle operator / (double divisor) const;
    angle& operator /= (double divisor);

    double operator / (const angle& divisor) const;

    bool operator == (const angle& a) const;
    bool operator != (const angle& a) const;
    bool operator < (const angle& a) const;
    bool operator > (const angle& a) const;
    bool operator <= (const angle& a) const;
    bool operator >= (const angle& a) const;

    private:

    angle(int64_t us, bool);
    int64_t _micros;

    static const uint64_t us_ms = 1000;
    static const uint64_t us_sec = us_ms * 1000;
    static const uint64_t us_min = us_sec * 60;
    static const uint64_t us_deg = us_min * 60;
    static const uint64_t us_rad = us_deg * 180/M_PI;
    static const uint64_t us_rev = us_deg * 360;
};

template <typename T> angle angle::operator * (T scalar) const
{
    return angle(_micros * scalar, true);
}

template <typename T> angle& angle::operator *= (T scalar)
{
    _micros *= scalar;
    return *this;
}

template <typename T> angle operator * (T scalar, const angle& a)
{
    return angle::microseconds(a.micros() * scalar);
}

template <typename T> angle angle::radians(T rads)
{
    return angle(rads * us_rad, true);
}

template <typename T> angle angle::revolutions(T revs)
{
    return angle(revs * us_rev, true);
}

template <typename T> angle angle::degrees(T degs)
{
    return angle(degs * us_deg, true);
}

template <typename T> angle angle::minutes(T mins)
{
    return angle(mins * us_min, true);
}

template <typename T> angle angle::seconds(T secs)
{
    return angle(secs * us_sec, true);
}

template <typename T> angle angle::milliseconds(T ms)
{
    return angle(ms * us_ms, true);
}

template <typename T> angle angle::microseconds(T us)
{
    return angle(us, true);
}

std::ostream& operator << (std::ostream& os, const angle& a);

namespace angle_literals
{

angle operator "" _rad(unsigned long long radians);

angle operator "" _rad(long double radians);

angle operator "" _deg(unsigned long long degrees);

angle operator "" _deg(long double degrees);

angle operator "" _rev(unsigned long long revolutions);

angle operator "" _rev(long double revolutions);

angle operator "" _min(unsigned long long minutes);

angle operator "" _min(long double minutes);

angle operator "" _sec(unsigned long long seconds);

angle operator "" _sec(long double seconds);

angle operator "" _ms(unsigned long long ms);

angle operator "" _ms(long double ms);

angle operator "" _us(unsigned long long ms);

angle operator "" _us(long double ms);

} // namespace uav::angle_literals

} // namespace uav

#endif // ANGLE_H
