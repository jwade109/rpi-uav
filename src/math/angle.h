#ifndef ANGLE_H
#define ANGLE_H

#include <iostream>
#include <cmath>

namespace uav
{

class angle
{
    public:

    static angle radians(double rads);
    static angle rotations(double rots);
    static angle degrees(double degs);
    static angle minutes(double mins);
    static angle seconds(double secs);
    static angle milliseconds(uint64_t ms);
    static angle microseconds(uint64_t us);

    angle();
    angle(int64_t us);
    angle(const angle& a);
    
    double amic(

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

    uint64_t milliarc;
};

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
