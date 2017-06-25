#include <Kalman.h>
#include <Eigen/Dense>

#include <iostream>
using namespace std;

using namespace Eigen;

KalmanFilter::KalmanFilter(size_t measurements, size_t states)
{
    M = measurements;
    N = states;
    X = VectorXd::Zero(N);
    P = MatrixXd::Identity(N, N);
    F = MatrixXd::Identity(N, N);
    H = MatrixXd::Identity(M, N);
    R = MatrixXd::Identity(M, M) * 0.5;
    Q = MatrixXd::Identity(N, N) * 0.1;
}

VectorXd KalmanFilter::step(const VectorXd& Z)
{
    if (Z.rows() != M) return VectorXd::Zero(M);

    X = F * X;
    P = F * P * F.transpose() + Q;
    MatrixXd G = (P * H.transpose()) * (H * P * H.transpose() + R).inverse();
    X = X + G * (Z - H * X);
    P = (MatrixXd::Identity(N, N) - G * H) * P;
    return X;
}
