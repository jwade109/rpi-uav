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

class running_variance
{
    public:

    double value;

    running_variance();

    double step(double sample);

    private:

    unsigned num_samples;
    double S;
    running_average mean;
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
        return step(sample, 1.0/freq);
    }

    double step(double sample, double dx)
    {
        if (samples <= N) samples++;
        double slope = d_dx(sample, dx);
        double last = current;
        current = slope;
        if (samples <= N) return 0;
        if (dx == 0) return 0;
        else return (value = (slope - last) / dx);
    }

    double operator () (double x) { return step(x); }

    double operator () (double x, double dx) { return step(x, dx); }

    void reset()
    {
        value = 0;
        current = NAN;
        samples = 0;
        d_dx.reset();
    }

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
    double step(double sample, double) { return sample; }
    double operator () (double x) { return x; }
    double operator () (double x, double) { return x; }
    void reset() { }
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

    void reset()
    {
        value = 0;
        sum_dx.reset();
    }

    private:

    integral<N-1> sum_dx;
};

template <> class integral<0>
{
    public:

    integral(uint8_t) { };

    double step(double sample) { return sample; }
    double operator () (double x) { return x; }
    void reset() { }
};

class range_accumulator
{
    public:

    double value;
    const double range;

    range_accumulator(double range);
    range_accumulator(double min, double max);

    double step(double bounded);

    private:

    bool first;
    double previous;
    derivative<1> diff;
};

} // namespace uav

#endif // FILTERS_H
