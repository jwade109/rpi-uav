#ifndef FILTERS_H
#define FILTERS_H

class RateLimiter
{
    public:

    RateLimiter(float max, float initial = 0);
    ~RateLimiter();

    float step(float sample, float dt);
    float get();

    private:

    float value;
    float max;
};

class LowPassFilter
{
    public:

    LowPassFilter(float alpha, float initial = 0);
    ~LowPassFilter();

    float step(float sample);
    float get();

    private:

    float a;
    float accumulator;
};

#endif // FILTERS_H
