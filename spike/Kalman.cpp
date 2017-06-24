// Kalman.cpp
#include "Kalman.h"
#include "Matrix.h"

KalmanFilter::KalmanFilter(size_t measurements, size_t states)
{
    M = measurements;
    N = states;
    X = Matrix(N, 1);
    P = Matrix::identity(N, N);
    F = Matrix::identity(N, N);
    H = Matrix::identity(M, N);
    R = Matrix::identity(M, M) * 0.5;
    Q = Matrix::identity(N, N) * 0.1;
}

KalmanFilter::~KalmanFilter()
{
}

Matrix KalmanFilter::step(const Matrix& Z)
{
    X = F * X;
    P = F * P * F.transpose() + Q;
    Matrix G = (P * H.transpose()) * (H * P * H.transpose() + R).inverse();
    X = X + G * (Z - H * X);
    P = (Matrix::identity(N, N) - G * H) * P;
    return X;
}
