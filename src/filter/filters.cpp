#include "filters.h"

namespace uav
{

low_pass::low_pass(double rc) : value(0), rc(rc), first(true) { }

double low_pass::step(double sample, double dt)
{
    if (first) { value = sample; first = false; }
    if (dt == 0) return value;
    double a = dt/(rc + dt);
    return (value = a * sample + (1 - a) * value);
}

high_pass::high_pass(double rc) : value(0), rc(rc), lpf(rc) { }

double high_pass::step(double sample, double dt)
{
    if (dt == 0) return value;
    lpf.rc = rc;
    return (value = sample - lpf.step(sample, dt));
}

running_average::running_average() : value(0), num_samples(0) { }

double running_average::step(double sample)
{
    if (num_samples++ == 0) return (value = sample);
    return (value = value * ((1.0 * (num_samples - 1))/num_samples) +
        sample/num_samples);
}

moving_average::moving_average(unsigned num_samples) :
    value(0), num_samples(num_samples) { }

double moving_average::step(double sample)
{
    samples.push_back(sample);
    if (samples.size() <= num_samples)
    {
        return value = ravg.step(sample);
    }
    double to_remove = samples.front();
    samples.pop_front();
    return (value = value + (sample - to_remove)/num_samples);
}

running_variance::running_variance() :
    value(0), num_samples(0), S(0) { }

double running_variance::step(double sample)
{
    num_samples++;
    double m_old = mean.value;
    mean.step(sample);
    S += (sample - mean.value) * (sample - m_old);
    if (num_samples == 1) return 0;
    return value = S/(num_samples - 1);
}

range_accumulator::range_accumulator(double min, double max) :
    range_accumulator(max - min) { }

range_accumulator::range_accumulator(double range) :
    value(0), range(range), first(true), previous(0) { }

double range_accumulator::step(double bounded)
{
    double d = diff.step(bounded);
    if (first) { first = false; return value = bounded; }
    double conj = d > 0 ? d - range : d + range;
    return value += std::abs(d) < std::abs(conj) ? d : conj;
}

double range_accumulator::operator () (double b)
{
    return step(b);
}

} // namespace uav
