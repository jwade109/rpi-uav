#ifndef KALMAN_H
#define KALMAN_H

#include <Eigen/Dense>

using namespace Eigen;

class KalmanFilter {
    
    public:
    
        VectorXd X;
        MatrixXd P;
        MatrixXd F;
        MatrixXd H;
        MatrixXd R;
        MatrixXd Q;
        size_t M;
        size_t N;
        
        KalmanFilter(size_t measurements, size_t states);
        VectorXd step(const VectorXd& Z);
};

#endif
