#include <iostream>
#include <chrono>
#include <thread>
#include <random>

#include <filters.h>

auto seed = std::chrono::steady_clock::now().time_since_epoch().count();
std::default_random_engine gen(seed);
std::normal_distribution<double> gaussian(0.0,1.0);

class mass_estimator
{
    public:

    double value;
    const uint8_t freq;
    const double accel_rc, mavg_sample_time, epsilon;

    mass_estimator(uint8_t freq);
    mass_estimator(uint8_t freq, double accel_rc,
                   double mavg_sample_time, double epsilon_detect);

    double step(double Fz, double az);

    private:

    uav::low_pass accel_lpf;
    uav::running_average global;
    uav::moving_average moving;
    bool diverged = false;

    const double dt;
};

mass_estimator::mass_estimator(uint8_t freq) :
    mass_estimator(freq, 0.2, 1.5, 0.2) { }

mass_estimator::mass_estimator(uint8_t f, double acc_rc,
        double mavg_st, double eps) :
    value(0), freq(f), accel_rc(acc_rc), mavg_sample_time(mavg_st),
    epsilon(eps), accel_lpf(acc_rc),
    moving(freq * mavg_st), diverged(false), dt(1.0/f) { }

double mass_estimator::step(double Fz, double az)
{
    double smooth_accel = accel_lpf.step(az, dt);
    if (isnan(smooth_accel)) return value;
    double m_hat = Fz/(smooth_accel + 9.81);

    double local_m_hat = moving.step(m_hat);
    double global_m_hat = global.step(m_hat);

    bool large_gap = std::abs(global_m_hat - local_m_hat) > epsilon;
    if (large_gap && !diverged) global = uav::running_average();
    diverged = large_gap;

    return (value = diverged ?
            std::max(global_m_hat, local_m_hat) : global_m_hat);
}

int main(int argc, char** argv)
{
    mass_estimator me(50);

    std::cout << "i\tmass\tacc\tm_hat"
        << std::fixed << std::endl;

    double true_accel = 4.3; // m/s^2
    double true_force = 23.1; // N

    uint64_t i = 0;
    while (i < 10000)
    {
        if (i == 2000) true_accel = 0.7;
        if (i == 4000) { true_accel = 7.4; true_force = 9.0; };
        if (i == 6000) { true_accel = 0; true_force = 0; };
        if (i == 8000) { true_accel = -6; true_force = 1; };

        double true_mass = true_force/(true_accel + 9.81);
        double noisy_accel = true_accel + gaussian(gen) * 2;

        double m_hat = me.step(true_force, noisy_accel);

        std::cout << i++ << "\t" << true_mass << "\t"
            << noisy_accel << "\t" << m_hat << std::endl;
    }
    return 0;
}
