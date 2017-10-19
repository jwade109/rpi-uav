#ifndef FILTERS_H
#define FILTERS_H

#include <cstdint>
#include <cmath>
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
    unsigned num_samples;

    moving_average(unsigned num_samples);

    double step(double sample);

    private:

    std::deque<double> samples;
};

template <uint8_t N> class derivative
{
    public:

    double value;
    const uint8_t freq;
    const double dt;
    const static int order = N;

    derivative<N>(uint8_t freq = 1) : value(0), freq(freq),
        dt(1.0/freq), current(NAN), samples(0), d_dx(freq) { };

    double step(double sample)
    {
        if (samples <= N) samples++;
        double slope = d_dx(sample);
        double last = current;
        current = slope;
        if (samples <= N) return 0;
        else return (value = (slope - last) * freq);
    }

    double operator () (double x) { return step(x); }

    private:

    double current;
    uint8_t samples;
    derivative<N-1> d_dx;
};

template <> class derivative<0>
{
    public:

    derivative(uint8_t) { };

    double step(double sample) { return sample; }
    double operator () (double x) { return x; }
};

template <uint8_t N> class integral
{
    public:

    double value;
    const uint8_t freq;
    const double dt;
    const static int order = N;

    integral<N>(uint8_t freq = 1) : value(0), freq(freq),
        dt(1.0/freq), sum_dx(freq) { }

    double step(double sample)
    {
        return (value += sum_dx(sample) * dt);
    }

    double operator () (double x) { return step(x); }

    private:

    integral<N-1> sum_dx;
};

template <> class integral<0>
{
    public:

    integral(uint8_t) { };

    double step(double sample) { return sample; }
    double operator () (double x) { return x; }
};

} // namespace uav

#endif // FILTERS_H
