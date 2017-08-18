#include <filters.h>

RateLimiter::RateLimiter(double maxrate, double initial):
    max(maxrate), value(initial) { }

RateLimiter::~RateLimiter() { }

double RateLimiter::step(double sample, double dt)
{
    double rate = (sample - value)/dt;
    if (rate > max)
        value += max * dt;
    else if (rate < -max)
        value -= max * dt;
    else value = sample;
    return value;
}

double RateLimiter::get()
{
    return value;
}

LowPassFilter::LowPassFilter(double rc, double initial):
    rc(rc), accumulator(initial) { }

LowPassFilter::~LowPassFilter() { }

double LowPassFilter::step(double sample, double dt)
{
    double a = dt/(rc + dt);
    accumulator = a * sample + (1 - a) * accumulator;
    return accumulator;
}

double LowPassFilter::get()
{
    return accumulator;
}
