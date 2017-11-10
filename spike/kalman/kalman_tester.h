#ifndef KALMAN_TESTER_H
#define KALMAN_TESTER_H

#include <iomanip>
#include <tuple>
#include <vector>
#include <string>
#include <sstream>

#include "kalman.h"

template <int M, int N, typename rep>
std::string printmat(const Eigen::Matrix<rep, M, N>& m)
{
    std::stringstream s;
    s << std::fixed;
    s << std::setprecision(4);
    for (auto i = m.data(); i < m.data() + m.size(); i++)
        s << std::setw(9) << *i << " ";
    return s.str();
}

template <size_t M, size_t N, size_t U, typename rep>
std::string printfilter(const kalman<M, N, U, rep>& k)
{
    std::stringstream s;
    s << "x: " << printmat(k.x) << std::endl
      << "z: " << printmat(k.z) << std::endl
      << "u: " << printmat(k.u) << std::endl
      << "P: " << printmat(k.P) << std::endl
      << "K: " << printmat(k.K) << std::endl
      << "A: " << printmat(k.A) << std::endl
      << "B: " << printmat(k.B) << std::endl
      << "H: " << printmat(k.H) << std::endl
      << "R: " << printmat(k.R) << std::endl
      << "Q: " << printmat(k.Q) << std::endl;
    return s.str();
}

template <size_t M, size_t N, size_t U, typename rep>
class kalman_tester
{
    public:

    kalman_tester(kalman<M, N, U, rep>& kf);

    void step(const Eigen::Matrix<rep, M, 1>& meas,
              const Eigen::Matrix<rep, N, 1>& real);

    bool converged();

    const std::vector<Eigen::Matrix<rep, N, 1>>& results();

    Eigen::Matrix<rep, N, 1> rms();

    private:

    Eigen::Matrix<rep, N, M> last_G;
    kalman<M, N, rep>& filter;
    std::vector<Eigen::Matrix<rep, N, 1>> error;
};

template <size_t M, size_t N, typename rep>
kalman_tester<M, N, rep>::kalman_tester(kalman<M, N, rep> *kf)
{
    if (kf != 0)
    {
        filter = kf;
        last_G = kf->G;
    }
}

template <size_t M, size_t N, typename rep>
void kalman_tester<M, N, rep>::step(const Eigen::Matrix<rep, M, 1>& meas,
                                    const Eigen::Matrix<rep, N, 1>& real)
{
    last_G = filter->G;
    auto calc = filter->step(meas);
    auto residual = calc - real;
    error.push_back(residual);
}

template <size_t M, size_t N, typename rep>
bool kalman_tester<M, N, rep>::converged()
{
    return std::abs(last_G.norm() - filter->G.norm()) < 0.002;
}

template <size_t M, size_t N, typename rep>
const std::vector<Eigen::Matrix<rep, N, 1>>& kalman_tester<M, N, rep>::results()
{
    return error;
}

template <size_t M, size_t N, typename rep>
Eigen::Matrix<rep, N, 1> kalman_tester<M, N, rep>::rms()
{
    Eigen::Matrix<rep, N, 1> ret = Eigen::Matrix<rep, N, 1>::Zero();
    for (auto& e : error)
        ret += e.cwiseProduct(e);
    ret /= error.size();
    ret = ret.cwiseSqrt();
    return ret;
}

#endif // KALMAN_TESTER_H
