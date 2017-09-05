#ifndef KALMAN_H
#define KALMAN_H

#include <Eigen/Dense>

template <size_t M, size_t N, typename rep = double>
class kalman
{
    public:

    Eigen::Matrix<rep, N, 1> X;
    Eigen::Matrix<rep, N, N> P;
    Eigen::Matrix<rep, N, N> F;
    Eigen::Matrix<rep, M, N> H;
    Eigen::Matrix<rep, M, M> R;
    Eigen::Matrix<rep, N, N> Q;

    kalman();
    kalman(const Eigen::Matrix<rep, N, 1>& initial);
    const Eigen::Matrix<rep, N, 1>& step(const Eigen::Matrix<rep, M, 1>& Z);
};

template <size_t M, size_t N, typename rep>
kalman<M, N, rep>::kalman() { }

template <size_t M, size_t N, typename rep>
kalman<M, N, rep>::kalman(const Eigen::Matrix<rep, N, 1>& initial) :
    X(initial) { }

template <size_t M, size_t N, typename rep>
const Eigen::Matrix<rep, N, 1>& kalman<M, N, rep>::
    step(const Eigen::Matrix<rep, M, 1>& Z)
{
    X = F * X;
    P = F * P * F.transpose() + Q;
    Eigen::MatrixXd G = (P * H.transpose()) *
        (H * P * H.transpose() + R).inverse();
    X = X + G * (Z - H * X);
    P = (Eigen::MatrixXd::Identity(N, N) - G * H) * P;
    return X;
}

#endif // KALMAN_H
