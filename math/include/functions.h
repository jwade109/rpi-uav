#ifndef FUNCTIONS_H
#define FUNCTIONS_H

#include <Eigen/Core>
#include <Eigen/Geometry>
#include "angle.h"

namespace uav
{

double altitude(double p_atm);

angle target_azimuth(const angle& current, const angle& desired);

Eigen::Matrix3d angle2matrix
    (const angle& alpha, const angle& beta, const angle& gamma);

Eigen::Matrix3d deg2matrix(const Eigen::Vector3d& deg);

Eigen::Matrix3d rad2matrix(const Eigen::Vector3d& rad);

Eigen::Quaterniond angle2quat
    (const angle& alpha, const angle& beta, const angle& gamma);

Eigen::Quaterniond deg2quat(const Eigen::Vector3d& deg);

Eigen::Quaterniond rad2quat(const Eigen::Vector3d& rad);

} // namespace uav

#endif
