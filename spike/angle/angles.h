#ifndef ANGLE_H
#define ANGLE_H

#include <iostream>
#include <cmath>

namespace uav
{

class angle
{
    public:

    angle(double rads);
    angle(const angle& a);

    double rad() const;
    double deg() const;

    angle& operator = (const angle& a);
    angle operator + (const angle& a) const;
    angle operator - (const angle& a) const;
    angle& operator += (const angle& a);
    angle& operator -= (const angle& a);

    template <typename T> angle operator * (T scalar) const;
    template <typename T> angle operator / (T divisor) const;
    template <typename T> angle& operator *= (T scalar);
    template <typename T> angle& operator /= (T scalar);

    bool operator == (const angle& a) const;
    bool operator != (const angle& a) const;
    bool operator < (const angle& a) const;
    bool operator > (const angle& a) const;
    bool operator <= (const angle& a) const;
    bool operator >= (const angle& a) const;

    operator double () const;

    private:

    double radians;
};

std::ostream& operator << (std::ostream& os, const angle& a);

template <typename T> angle angle::operator * (T scalar) const
{
    return angle(radians * scalar);
}

template <typename T> angle angle::operator / (T divisor) const
{
    return angle(radians / divisor);
}

template <typename T> angle& angle::operator *= (T scalar)
{
    return (*this = *this * scalar);
}

template <typename T> angle& angle::operator /= (T divisor)
{
    return (*this = *this / divisor);
}

namespace angle_literals
{

angle operator "" _rad(unsigned long long radians);

angle operator "" _deg(unsigned long long degrees);

angle operator "" _rad(long double radians);

angle operator "" _deg(long double degrees);

} // namespace uav::angle_literals

} // namespace uav

#endif // ANGLE_H
