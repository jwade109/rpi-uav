// Kalman.h
#ifndef KALMAN_H
#define KALMAN_H

#include "Matrix.h"

class KalmanFilter {
    
    public:
    
        Matrix X;
        Matrix P;
        Matrix F;
        Matrix H;
        Matrix R;
        Matrix Q;
        size_t M;
        size_t N;
        
        KalmanFilter(size_t measurements, size_t states);
        
        ~KalmanFilter();
        
        Matrix step(const Matrix& Z);
};

#endif
