#include <iostream>
#include <chrono>
#include <random>

#include <filters.h>

std::uniform_real_distribution<double> uniform(0, 1);
std::default_random_engine gen;

int main()
{
    uav::low_pass lpf(2);
    uav::high_pass hpf(2);
    uav::moving_average mavg(30);
    uav::running_average ravg;

    std::cout << "i,r,lpf,hpf,mavg,ravg" << std::endl;
    for (int i = 0; i < 1000; i++)
    {
        double r = 3 + uniform(gen);
        std::cout << i << "," << r << ","
            << lpf.step(r, 0.05) << ","
            << hpf.step(r, 0.05) << ","
            << mavg.step(r) << ","
            << ravg.step(r) << std::endl;
    }
    return 0;
}
