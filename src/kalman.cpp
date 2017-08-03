#include <kalman.h>
#include <Eigen/Dense>

KalmanFilter::KalmanFilter(size_t measurements, size_t states)
{
    M = measurements;
    N = states;
    X = Eigen::VectorXd::Zero(N);
    P = Eigen::MatrixXd::Identity(N, N);
    F = Eigen::MatrixXd::Identity(N, N);
    H = Eigen::MatrixXd::Identity(M, N);
    R = Eigen::MatrixXd::Identity(M, M) * 0.5;
    Q = Eigen::MatrixXd::Identity(N, N) * 0.1;
}

Eigen::VectorXd KalmanFilter::step(const Eigen::VectorXd& Z)
{
    if (Z.rows() != M) return Eigen::VectorXd::Zero(M);

    X = F * X;
    P = F * P * F.transpose() + Q;
    Eigen::MatrixXd G = (P * H.transpose()) * (H * P * H.transpose() + R).inverse();
    X = X + G * (Z - H * X);
    P = (Eigen::MatrixXd::Identity(N, N) - G * H) * P;
    return X;
}
