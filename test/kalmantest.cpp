#include <iostream>
#include <time.h>

#include <Kalman.h>
#include <timeutil.h>

using namespace std;

int main(int argc, char* argv[])
{
    srand(time(0));
    const size_t M = 2, N = 2;
    KalmanFilter kf(M, N);

    kf.R = MatrixXd::Identity(M, M) * 10;

    cout << "Measurement\t\tFiltered\t\tError\n";
    cout << "----------------------------------\n";

    for (int i = 0; i < 1000; i++)
    {
        VectorXd z = VectorXd::Constant(M, 10) + VectorXd::Random(M);
        VectorXd x = kf.step(z);

	cout << z(0) << "\t" << z(1) << "\t\t" << x(0) << "\t" << x(1);
        cout << "\t\t" << 10 - x(0) << "\t" << 10 - x(1) << endl;

	waitfor(100, milli);
    }
    return 0;
}

/*

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
        MatrixXd step(const MatrixXd& Z);
};

#endif

*/
