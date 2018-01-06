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
    return rad2matrix((M_PI/180) * deg);
}

Eigen::Matrix3d rad2matrix(const Eigen::Vector3d& rad)
{
    const auto X = Eigen::Vector3d::UnitX(),
               Y = Eigen::Vector3d::UnitY(),
               Z = Eigen::Vector3d::UnitZ();
    auto rot1 = Eigen::AngleAxisd(rad(0), Z);
    auto Xp = rot1 * X, Yp = rot1 * Y;
    auto rot2 = Eigen::AngleAxisd(rad(1), Xp);
    auto Ypp = rot2 * Yp, Zp = rot2 * Z;
    auto rot3 = Eigen::AngleAxisd(rad(2), Ypp);
    auto Xpp = rot3 * Xp, Zpp = rot3 * Zp;
    Eigen::Matrix3d ret;
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

Eigen::Vector3d matrix2deg(const Eigen::Matrix3d& mat)
{
    return (180/M_PI) * matrix2rad(mat);
}

Eigen::Vector3d matrix2rad(const Eigen::Matrix3d& mat)
{
    Eigen::Vector3d Xpp = mat.col(0), Ypp = mat.col(1),
        Zpp = mat.col(2),
        Yp = Eigen::Vector3d(Ypp(0), Ypp(1), 0).normalized();

    auto Xp = Yp.cross(Eigen::Vector3d::UnitZ()), Zp = Xp.cross(Ypp);

    double alpha = atan2(Xp.y(), Xp.x());
    double beta = asin(Ypp.z());
    double gamma = atan2(Xp.dot(Zpp), Zp.dot(Zpp));
    if (alpha < 0) alpha += 2*M_PI;
    return {alpha, beta, gamma};
}

Eigen::Vector3d quat2deg(const Eigen::Quaterniond& quat)
{
    return matrix2deg(quat.normalized().toRotationMatrix());
}

Eigen::Vector3d quat2rad(const Eigen::Quaterniond& quat)
{
    return matrix2rad(quat.normalized().toRotationMatrix());
}

} // namespace uav
