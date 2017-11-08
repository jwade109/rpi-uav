#ifndef COORDINATE_H
#define COORDINATE_H

#include <iostream>
#include <Eigen/Core>

#include "angle.h"

namespace uav
{

class coordinate
{
    public:

    coordinate();
    coordinate(const coordinate& c);
    coordinate(const angle& N, const angle& E, double alt = 0);
    coordinate(double dN, double dE, double alt = 0);

    template <typename S, typename T, typename U, typename V,
              typename W, typename X, typename Y, typename Z>
    coordinate(S dN, T mN, U sN, V msN,
               W dE, X mE, Y sE, Z msE, double alt = 0);

    const angle& latitude() const;
    angle& latitude();
    const angle& longitude() const;
    angle& longitude();
    double altitude() const;
    double& altitude();

    coordinate& operator = (const coordinate& c);
    coordinate operator + (const Eigen::Vector3d& v) const;
    coordinate operator - (const Eigen::Vector3d& v) const;
    Eigen::Vector3d operator - (const coordinate& c) const;
    coordinate& operator += (const Eigen::Vector3d& v);
    coordinate& operator -= (const Eigen::Vector3d& v);

    bool operator == (const coordinate& c) const;

    private:

    angle _latitude, _longitude;
    double _altitude;
};

std::ostream& operator << (std::ostream& os, const coordinate& c);

template <typename S, typename T, typename U, typename V,
          typename W, typename X, typename Y, typename Z>
coordinate::coordinate(S dN, T mN, U sN, V msN,
                       W dE, X mE, Y sE, Z msE, double alt) :
    coordinate(angle::degrees(dN) + angle::minutes(mN) +
               angle::seconds(sN) + angle::milliseconds(msN),
               angle::degrees(dE) + angle::minutes(mE) +
               angle::seconds(sE) + angle::milliseconds(msE), alt) { }

} // namespace uav

#endif // COORDINATE_H
