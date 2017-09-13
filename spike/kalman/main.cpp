#include <iostream>
#include <iomanip>
#include <chrono>
#include <thread>
#include <string>
#include <sstream>
#include <vector>
#include <time.h>
#include <signal.h>
#include <math.h>

#include "kalman.h"
#include "kalman_tester.h"

using namespace std;
using namespace std::chrono;
using namespace Eigen;

bool cont = true;

void sigint(int signal)
{
    cont = false;
}

int main(int argc, char* argv[])
{
    signal(SIGINT, sigint);

    srand(time(0));
    const size_t M = 2, N = 2;
    kalman<M, N, double> kf;

    kf.R *= 3;
    kf.Q *= 0.2;
    kf.P *= 100;
    kf.H(0,1) = kf.H(1,0) = 0;

    kalman_tester<M, N, double> kt(&kf);

    int max = 1000;
    for (int i = 0; i < max & cont; i++)
    {
        system("clear");
        std::cout << i << "/" << max << std::endl;
        std::cout << "Converged: " << kf.converged(0.0001) << std::endl;
        std::cout << printfilter(kf) << std::endl;

        auto real = Matrix<double, N, 1>::Constant(10);
        auto measure = Matrix<double, M, 1>::Constant(10) +
            Matrix<double, M, 1>::Random();
        kt.step(measure, real);

        std::this_thread::sleep_for(milliseconds(50));
    }

    std::cout << "Done.\nRMS: " << printmat(kt.rms()) << std::endl;

    return 0;
}
