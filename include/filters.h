#ifndef FILTERS_H
#define FILTERS_H

class RateLimiter
{
    public:

    RateLimiter(double max, double initial = 0);
    ~RateLimiter();

    double step(double sample, double dt);
    double get();

    private:

    double max;
    double value;
};

class LowPassFilter
{
    public:

    LowPassFilter(double rc, double initial = 0);
    ~LowPassFilter();

    double step(double sample, double dt);
    double get();

    private:

    double rc;
    double accumulator;
};

#endif // FILTERS_H
