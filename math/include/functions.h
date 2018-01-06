#ifndef FUNCTIONS_H
#define FUNCTIONS_H

#include <Eigen/Core>
#include <Eigen/Geometry>
#include "angle.h"

namespace uav
{

double altitude(double p_atm);

angle target_azimuth(const angle& current, const angle& desired);

// conversions from Tait-Bryan z-x'-y'' angles to quat/rotation matrix

Eigen::Matrix3d angle2matrix
    (const angle& alpha, const angle& beta, const angle& gamma);

Eigen::Matrix3d deg2matrix(const Eigen::Vector3d& deg);

Eigen::Matrix3d rad2matrix(const Eigen::Vector3d& rad);

Eigen::Quaterniond angle2quat
    (const angle& alpha, const angle& beta, const angle& gamma);

Eigen::Quaterniond deg2quat(const Eigen::Vector3d& deg);

Eigen::Quaterniond rad2quat(const Eigen::Vector3d& rad);

// conversions from rotation matrix to Tait-Bryan angles

Eigen::Vector3d matrix2deg(const Eigen::Matrix3d& mat);

Eigen::Vector3d matrix2rad(const Eigen::Matrix3d& mat);

// conversions from quaternion to Tait-Bryan angles

Eigen::Vector3d quat2deg(const Eigen::Quaterniond& quat);

Eigen::Vector3d quat2rad(const Eigen::Quaterniond& quat);

} // namespace uav

#endif
