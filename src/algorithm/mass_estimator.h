#ifndef MASS_ESTIMATOR_H
#define MASS_ESTIMATOR_H

#include <uav/filter>

namespace uav
{

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

} // namespace uav

#endif // MASS_ESTIMATOR_H
