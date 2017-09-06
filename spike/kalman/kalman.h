#ifndef KALMAN_H
#define KALMAN_H

#include <Eigen/Dense>

template <size_t M, size_t N, typename rep = double>
class kalman
{
    public:

    Eigen::Matrix<rep, N, 1> X;
    Eigen::Matrix<rep, M, 1> Z;
    Eigen::Matrix<rep, N, N> P;
    Eigen::Matrix<rep, N, M> G;
    Eigen::Matrix<rep, N, N> F;
    Eigen::Matrix<rep, M, N> H;
    Eigen::Matrix<rep, M, M> R;
    Eigen::Matrix<rep, N, N> Q;

    kalman();
    kalman(const Eigen::Matrix<rep, N, 1>& initial);
    const Eigen::Matrix<rep, N, 1>& step(const Eigen::Matrix<rep, M, 1>& Z);
};

// default constructor
template <size_t M, size_t N, typename rep>
kalman<M, N, rep>::kalman() :
    kalman(Eigen::Matrix<rep, N, 1>::Zero())
{ }

// constructor with initial state
template <size_t M, size_t N, typename rep>
kalman<M, N, rep>::kalman(const Eigen::Matrix<rep, N, 1>& initial) :
    X(initial),
    Z(Eigen::Matrix<rep, M, 1>::Zero()),
    P(Eigen::Matrix<rep, N, N>::Identity()),
    F(Eigen::Matrix<rep, N, N>::Identity()),
    H(Eigen::Matrix<rep, M, N>::Constant(1)),
    R(Eigen::Matrix<rep, M, M>::Identity()),
    Q(Eigen::Matrix<rep, N, N>::Identity())
{ }

template <size_t M, size_t N, typename rep>
const Eigen::Matrix<rep, N, 1>& kalman<M, N, rep>::
    step(const Eigen::Matrix<rep, M, 1>& meas)
{
    Z = meas;
    X = F * X;
    P = F * P * F.transpose() + Q;
    G = (P * H.transpose()) * (H * P * H.transpose() + R).inverse();
    X = X + G * (Z - H * X);
    P = (Eigen::MatrixXd::Identity(N, N) - G * H) * P;
    return X;
}

#endif // KALMAN_H
