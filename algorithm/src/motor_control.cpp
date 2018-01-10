// #ifndef MOTOR_CONTROL
// #define MOTOR_CONTROL

#include <iostream>
#include <iomanip>
#include <array>
#include <Eigen/Core>

#include "ins.h"

namespace uav
{

Eigen::Matrix<double, 4, 4> motor_matrix()
{
    Eigen::Matrix<double, 4, 4> mat;
    mat << 1, -1,  1, -1,
           1,  1,  1,  1,
           1, -1, -1,  1,
           1,  1, -1, -1;
    return mat;
}

Eigen::Vector4d limit_response(double hover, double pz,
    double palpha, double pbeta, double pgamma)
{
    const double att_min = 15; // min thrust alloc for attitude ctrl
    const double k_alpha = 0.2, k_beta = 0.4, k_gamma = 0.4;

    double att_pl = 100 - (hover + pz); // positive thrust lim for attitude ctrl
    double att_nl = 100 - att_pl; // negative thrust lim for attitude ctrl
    double att_el = std::min(att_pl, att_nl); // effective limit

    // requested thrust from att ctrl, and thrust allocated
    double att_req = std::abs(palpha) + std::abs(pbeta) + std::abs(pgamma);
    double att_alloc = std::min(std::max(att_el, att_min), att_req);

    double z_ul = 100 - (hover + att_alloc); // upper thrust lim z
    double z_ll = att_alloc - hover; // lower thrust lim z
    double pz_lim = std::min(std::max(pz, z_ll), z_ul);

    double palpha_lim = palpha,
           pbeta_lim = pbeta,
           pgamma_lim = pgamma;
    if (att_req > att_alloc) // must scale down attitude responses
    {
        auto sign = [](double x){ return x < 0 ? -1 : 1; };
        palpha_lim = std::abs(palpha) * (att_alloc/att_req) * k_alpha;
        pbeta_lim = std::abs(pbeta) * (att_alloc/att_req) * k_beta;
        pgamma_lim = std::abs(pgamma) * (att_alloc/att_req) * k_gamma;
        double sum = palpha_lim + pbeta_lim + pgamma_lim;
        palpha_lim *= (att_alloc/sum) * sign(palpha);
        pbeta_lim *= (att_alloc/sum) * sign(pbeta);
        pgamma_lim *= (att_alloc/sum) * sign(pgamma);
    }
    return {pz_lim, palpha_lim, pbeta_lim, pgamma_lim};
}

Eigen::Vector4d get_motors(double hover, const Eigen::Vector4d &resp)
{
    auto mat = motor_matrix();
    Eigen::Vector4d hover_v;
    hover_v << hover, hover, hover, hover;
    return mat * resp + hover_v;
}

Eigen::Vector4d get_response(double hover, const Eigen::Vector4d &motor)
{
    Eigen::Vector4d hover_v;
    hover_v << hover, hover, hover, hover;
    return motor_matrix().inverse() * (motor - hover_v);
}

} // namespace uav

int main()
{
    /*
    double h, z, a, b, g;
    std::cerr << "hover: " << std::flush;
    std::cin >> h;
    std::cerr << "pz: " << std::flush;
    std::cin >> z;
    std::cerr << "pa: " << std::flush;
    std::cin >> a;
    std::cerr << "pb: " << std::flush;
    std::cin >> b;
    std::cerr << "pg: " << std::flush;
    std::cin >> g;
    */

    int k = 0;

    std::cout << std::setprecision(2) << std::fixed;

    for (int h = 0; h <= 100; h+=10)
    for (int z = -120; z <= 120; z+=24)
    for (int a = -120; a <= 120; a+=24)
    for (int b = -120; b <= 120; b+=24)
    for (int g = -120; g <= 120; g+=24)
    {

    auto resp = uav::limit_response(h, z, a, b, g);
    auto motors = uav::get_motors(h, resp);
    auto resp2 = uav::get_response(h, motors);
    std::cout << std::setw(6) << k << " "
        << std::setw(6) << h << " " << std::setw(6) << z << " "
        << std::setw(6) << a << " " << std::setw(6) << b << " "
        << std::setw(6) << g << " | "
        << std::setw(6) << resp(0) << " " << std::setw(6) << resp(1) << " "
        << std::setw(6) << resp(2) << " " << std::setw(6) << resp(3) << " | "
        << std::setw(6) << motors(0) << " " << std::setw(6) << motors(1) << " "
        << std::setw(6) << motors(2) << " " << std::setw(6) << motors(3) << " | "
        << std::setw(6) << resp2(0) << " " << std::setw(6) << resp2(1) << " "
        << std::setw(6) << resp2(2) << " " << std::setw(6) << resp2(3) << std::endl;
    k++;
    }

    // std::cout << "----" << std::endl;
    // std::cout << "response:\n" << resp << std::endl;
    // std::cout << "motors:\n" << motors << std::endl;
    // std::cout << "response:\n" << uav::get_response(h, motors) << std::endl;
}

// #endif // MOTOR_CONTROL
