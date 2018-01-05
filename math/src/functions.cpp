#include "functions.h"

namespace uav
{

double altitude(double p_atm)
{
    return 44330 * (1.0 - pow(p_atm, 0.1903));
}

angle target_azimuth(const angle& current, const angle& target)
{
    return current;
}

Eigen::Matrix3d angle2matrix
    (const angle& alpha, const angle& beta, const angle& gamma)
{
    return rad2matrix({alpha, beta, gamma});
}

Eigen::Matrix3d deg2matrix(const Eigen::Vector3d& deg)
{
    return rad2matrix((M_PI/180.0) * deg);
}

Eigen::Matrix3d rad2matrix(const Eigen::Vector3d& rad)
{
    using namespace Eigen;
    const auto X = Vector3d::UnitX(),
               Y = Vector3d::UnitY(),
               Z = Vector3d::UnitZ();
    auto rot1 = AngleAxisd(rad(0), Z);
    auto Xp = rot1 * X, Yp = rot1 * Y;
    auto rot2 = AngleAxisd(rad(1), Xp);
    auto Ypp = rot2 * Yp, Zp = rot2 * Z;
    auto rot3 = AngleAxisd(rad(2), Ypp);
    auto Xpp = rot3 * Xp, Zpp = rot3 * Zp;
    Matrix3d ret;
    ret << Xpp, Ypp, Zpp;
    return ret;
}

Eigen::Quaterniond angle2quat
    (const angle& alpha, const angle& beta, const angle& gamma)
{
    return Eigen::Quaterniond(angle2matrix(alpha, beta, gamma));
}

Eigen::Quaterniond deg2quat(const Eigen::Vector3d& deg)
{
    return Eigen::Quaterniond(deg2matrix(deg));
}

Eigen::Quaterniond rad2quat(const Eigen::Vector3d& rad)
{
    return Eigen::Quaterniond(rad2matrix(rad));
}

} // namespace uav
