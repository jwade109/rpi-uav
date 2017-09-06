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

using namespace std;
using namespace std::chrono;
using namespace Eigen;

bool cont = true;

void sigint(int signal)
{
    cont = false;
}

template <int M, int N, typename rep>
const string println(const Matrix<rep, M, N>& m)
{
    stringstream s;
    s << fixed;
    s << setprecision(12);
    for (int i = 0; i < m.rows(); i++)
        for (int j = 0; j < m.cols(); j++)
            s << setw(15) << m(i, j) << " ";
    return s.str();
}

template <size_t M, size_t N, typename rep>
void print(const kalman<M, N, rep>& k)
{
    std::cout
        << "X: " << println(k.X) << std::endl
        << "Z: " << println(k.Z) << std::endl
        << "P: " << println(k.P) << std::endl
        << "G: " << println(k.G) << std::endl
        << "F: " << println(k.F) << std::endl
        << "H: " << println(k.H) << std::endl
        << "R: " << println(k.R) << std::endl
        << "Q: " << println(k.Q) << std::endl;
}

int main(int argc, char* argv[])
{
    signal(SIGINT, sigint);

    vector<double> raw1, raw2, error;

    srand(time(0));
    const size_t M = 2, N = 1;
    kalman<M, N, double> kf;

    kf.R *= 3;
    kf.Q *= 0.2;
    kf.P *= 100;

    std::cout << "[" << std::flush;

    int max = 10000, bars = max/80;
    for (int i = 0; i < max & cont; i++)
    {
        // system("clear");
        VectorXd z = VectorXd::Constant(M, 10) + VectorXd::Random(M);
        // print(kf);
        kf.step(z);

        raw1.push_back(kf.Z(0));
        raw2.push_back(kf.Z(1));
        error.push_back(kf.X(0));
        if (i % bars == 0) std::cout << "|" << std::flush;
        // this_thread::sleep_for(milliseconds(150));
    }
    std::cout << "]" << std::endl;

    double avg[3] = {0, 0, 0};
    for (double e : raw1)
        avg[0] += (e - 10) * (e - 10);
    for (double e : raw2)
        avg[1] += (e - 10) * (e - 10);
    for (double e : error)
        avg[2] += (e - 10) * (e - 10);
    for (double& e : avg)
        e = sqrt(e/raw1.size());

    std::cout << "Done.\nRMS: ";
    for (double e : avg)
        std::cout << e << " ";
    std::cout << std::endl;

    return 0;
}
