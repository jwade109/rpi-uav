#include <iostream>
#include <cmath>

template <uint8_t N> class derivative
{
    public:

    double value;
    const uint8_t freq;
    const double dt;
    const static int order = N;

    derivative<N>(uint8_t freq) : value(0), freq(freq),
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

int main()
{
    const int freq = 200;
    const double dt = 1.0/freq;

    auto y = [] (double x)
        { return x * x * (x - 0.2) * (x + 0.3) * (x - 0.74); };

    derivative<1> d1(freq);
    derivative<2> d2(freq);
    derivative<3> d3(freq);
    derivative<4> d4(freq);
    derivative<5> d5(freq);

    std::cout << "x\ty\td1\td2\td3\td4\td5"
        << std::fixed << std::endl;

    for (double x = -1; x < 1; x += dt)
    {
        std::cout << x << "\t" << y(x) << "\t"
            << d1(y(x)) << "\t"
            << d2(y(x)) << "\t"
            << d3(y(x)) << "\t"
            << d4(y(x)) << "\t"
            << d5(y(x)) << std::endl;
    }
    return 0;
}

