#ifndef KALMAN_H
#define KALMAN_H

#include <Eigen/Dense>

template <size_t M, size_t N, size_t U, typename rep = double>
class kalman
{
    public:

    using state = Eigen::Matrix<rep, M, 1>;
    using meas = Eigen::Matrix<rep, N, 1>;
    using control = Eigen::Matrix<rep, U, 1>;

    state x; // state estimate
    meas z; // last measurement
    control u; // last control vector

    Eigen::Matrix<rep, N, N> P; // error covariance
    Eigen::Matrix<rep, N, M> K; // kalman gain
    Eigen::Matrix<rep, N, N> A; // state transition
    Eigen::Matrix<rep, N, U> B; // control-input model
    Eigen::Matrix<rep, M, N> H; // observation model
    Eigen::Matrix<rep, M, M> R; // measurement noise
    Eigen::Matrix<rep, N, N> Q; // process noise

    kalman();
    kalman(const Eigen::Matrix<rep, N, 1>& initial);
    const Eigen::Matrix<rep, N, 1>&
        step(const Eigen::Matrix<rep, M, 1>& z,
             const Eigen::Matrix<rep, U, 1>& u =
             Eigen::Matrix<rep, U, 1>::Zero());
};

// default constructor
template <size_t M, size_t N, size_t U, typename rep>
kalman<M, N, U, rep>::kalman() :
    kalman(Eigen::Matrix<rep, N, 1>::Zero())
{ }

// constructor with initial state
template <size_t M, size_t N, size_t U, typename rep>
kalman<M, N, U, rep>::kalman(const Eigen::Matrix<rep, N, 1>& initial) :
    x(initial),
    z(Eigen::Matrix<rep, M, 1>::Zero()),
    u(Eigen::Matrix<rep, U, 1>::Zero()),
    P(Eigen::Matrix<rep, N, N>::Identity()),
    K(Eigen::Matrix<rep, N, M>::Zero()),
    A(Eigen::Matrix<rep, N, N>::Identity()),
    B(Eigen::Matrix<rep, N, U>::Zero()),
    H(Eigen::Matrix<rep, M, N>::Constant(1)),
    R(Eigen::Matrix<rep, M, M>::Identity()),
    Q(Eigen::Matrix<rep, N, N>::Identity())
{ }

template <size_t M, size_t N, size_t U, typename rep>
const Eigen::Matrix<rep, N, 1>& kalman<M, N, U, rep>::
    step(const Eigen::Matrix<rep, M, 1>& z,
         const Eigen::Matrix<rep, U, 1>& u)
{
    this->z = z;
    x = A*x + B*u;
    P = A * P * A.transpose() + Q;
    K = (P * H.transpose()) * (H * P * H.transpose() + R).inverse();
    x = x + K * (z - H * x);
    P = (Eigen::Matrix<rep, N, N>::Identity() - K * H) * P;
    return x;
}

#endif // KALMAN_H
