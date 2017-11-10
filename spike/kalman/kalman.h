#ifndef KALMAN_H
#define KALMAN_H

#include <Eigen/Dense>

template <size_t M, size_t N, size_t U, typename rep = double>
class kalman
{
    public:

    using state = Eigen::Matrix<rep, M, 1>;
    using measure = Eigen::Matrix<rep, N, 1>;
    using control = Eigen::Matrix<rep, U, 1>;

    state x; // state estimate
    measure z; // last measurement
    control u; // last control vector

    Eigen::Matrix<rep, N, N> P; // error covariance
    Eigen::Matrix<rep, N, M> K; // kalman gain
    Eigen::Matrix<rep, N, N> A; // state transition
    Eigen::Matrix<rep, N, U> B; // control-input model
    Eigen::Matrix<rep, M, N> H; // observation model
    Eigen::Matrix<rep, M, M> R; // measurement noise
    Eigen::Matrix<rep, N, N> Q; // process noise

    kalman(const state& initial) :
        x(initial),
        z(decltype(z)::Zero()),
        u(decltype(u)::Zero()),
        P(decltype(P)::Identity()),
        K(decltype(K)::Zero()),
        A(decltype(A)::Identity()),
        B(decltype(B)::Zero()),
        H(decltype(H)::Constant(1)),
        R(decltype(R)::Identity()),
        Q(decltype(Q)::Identity()) { }

    kalman() : kalman(state::Zero()) { }

    const state& predict(const control& u = control::Zero())
    {
        this->u = u;
        x = A*x + B*u;
        P = A * P * A.transpose() + Q;
        return x;
    }

    const state& update(const measure& z)
    {
        this->z = z;
        K = (P * H.transpose()) * (H * P * H.transpose() + R).inverse();
        x = x + K * (z - H * x);
        P = (Eigen::Matrix<rep, N, N>::Identity() - K * H) * P;
        return x;
    }
};

#endif // KALMAN_H
