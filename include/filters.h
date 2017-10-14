#ifndef FILTERS_H
#define FILTERS_H

#include <cstdint>
#include <deque>

namespace uav
{

class low_pass
{
    public:

    double value, rc;

    low_pass(double rc);

    double step(double sample, double dt);

    private:

    bool first;
};

class high_pass
{
    public:

    double value, rc;

    high_pass(double rc);

    double step(double sample, double dt);

    private:

    low_pass lpf;
};

class running_average
{
    public:

    double value;
    unsigned num_samples;

    running_average();

    double step(double sample);
};

class moving_average
{
    public:

    double value;
    uint8_t num_samples;

    moving_average(uint8_t num_samples);

    double step(double sample);

    private:

    std::deque<double> samples;
};

} // namespace uav

#endif // FILTERS_H
