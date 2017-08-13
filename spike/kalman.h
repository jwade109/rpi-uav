#ifndef KALMAN_H
#define KALMAN_H

#include <Eigen/Dense>

class KalmanFilter {
    
public:
    
    Eigen::VectorXd X;
    Eigen::MatrixXd P;
    Eigen::MatrixXd F;
    Eigen::MatrixXd H;
    Eigen::MatrixXd R;
    Eigen::MatrixXd Q;
    
    size_t M;
    size_t N;
        
    KalmanFilter(size_t measurements, size_t states);
    Eigen::VectorXd step(const Eigen::VectorXd& Z);
};

#endif // KALMAN_H
