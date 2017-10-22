#ifndef ANGLE_H
#define ANGLE_H

#include <iostream>
#include <cmath>

namespace uav
{

class angle
{
    public:

    angle();
    angle(double rads, int rots = 0);
    angle(const angle& a);

    double rad() const;
    double deg() const;
    int rot() const;

    angle operator - ();
    angle& operator = (const angle& a);
    angle operator + (const angle& a) const;
    angle operator - (const angle& a) const;
    angle& operator += (const angle& a);
    angle& operator -= (const angle& a);

    angle& operator = (double rads);

    angle operator * (double scalar) const;
    angle operator / (double divisor) const;
    angle& operator *= (double scalar);
    angle& operator /= (double divisor);

    double operator / (const angle& divisor) const;

    bool operator == (const angle& a) const;
    bool operator != (const angle& a) const;
    bool operator < (const angle& a) const;
    bool operator > (const angle& a) const;
    bool operator <= (const angle& a) const;
    bool operator >= (const angle& a) const;

    operator double () const;

    private:

    int rotations;
    double radians;
};

angle target_azimuth(angle current, angle desired);

std::ostream& operator << (std::ostream& os, const angle& a);

namespace angle_literals
{

angle operator "" _rad(unsigned long long radians);

angle operator "" _deg(unsigned long long degrees);

angle operator "" _rad(long double radians);

angle operator "" _deg(long double degrees);

} // namespace uav::angle_literals

} // namespace uav

#endif // ANGLE_H
