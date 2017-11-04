#include "mass_estimator.h"

namespace uav
{

mass_estimator::mass_estimator(uint8_t freq) :
    mass_estimator(freq, 0.2, 1.5, 0.2) { }

mass_estimator::mass_estimator(uint8_t f, double acc_rc,
    double mavg_st, double eps) :
    value(0), freq(f), accel_rc(acc_rc), mavg_sample_time(mavg_st),
    epsilon(eps), accel_lpf(acc_rc),
    moving(freq * mavg_st), diverged(false), dt(1.0/f) { }

double mass_estimator::step(double Fz, double az)
{
    if (az == 9.81) return value;
    double smooth_accel = accel_lpf.step(az, dt);
    double m_hat = Fz/(smooth_accel + 9.81);

    double local_m_hat = moving.step(m_hat);
    double global_m_hat = global.step(m_hat);

    bool large_gap = std::abs(global_m_hat - local_m_hat) > epsilon;
    if (large_gap && !diverged) global = uav::running_average();
    diverged = large_gap;

    return value = diverged ? local_m_hat : global_m_hat;
}

} // namespace uav
