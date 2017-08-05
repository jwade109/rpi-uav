#include <filters.h>

RateLimiter::RateLimiter(float maxrate, float initial):
    max(maxrate), value(initial) { }

RateLimiter::~RateLimiter() { }

float RateLimiter::step(float sample, float dt)
{
    float rate = (sample - value)/dt;
    if (rate > max)
        value += max * dt;
    else if (rate < -max)
        value -= max * dt;
    else value = sample;
    return value;
}

float RateLimiter::get()
{
    return value;
}

LowPassFilter::LowPassFilter(float alpha, float initial):
    a(alpha), accumulator(initial) { }

LowPassFilter::~LowPassFilter() { }

float LowPassFilter::step(float sample)
{
    accumulator = sample * a + accumulator * (1 - a);
    return accumulator;
}

float LowPassFilter::get()
{
    return accumulator;
}
