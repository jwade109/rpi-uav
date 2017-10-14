#include <filters.h>

uav::low_pass::low_pass(double rc) : value(0), rc(rc) { }

double uav::low_pass::step(double sample, double dt)
{
    static bool init(false);
    if (!init) { value = sample; init = true; }
    if (dt == 0) return value;
    double a = dt/(rc + dt);
    return (value = a * sample + (1 - a) * value);
}

uav::running_average::running_average() : value(0), num_samples(0) { }

double uav::running_average::step(double sample)
{
    if (num_samples++ == 0) return (value = sample);
    return (value = value * ((1.0 * num_samples)/(num_samples + 1)) +
        sample/(num_samples + 1));
}

uav::moving_average::moving_average(uint8_t num_samples) :
    value(0), num_samples(num_samples) { }

double uav::moving_average::step(double sample)
{
    samples.push_back(sample);
    if (samples.size() <= num_samples)
    {
        running_average ravg;
        for (double e : samples)
            ravg.step(e);
        return (value = ravg.value);
    }
    double to_remove = samples.front();
    samples.pop_front();
    return (value = value + (sample - to_remove)/num_samples);
}
