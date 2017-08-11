#include <filters.h>
#include <stdio.h>
#include <iostream>
#include <chrono>
#include <thread>

int main()
{
    using namespace std::chrono;

    float point = 150;
    RateLimiter rt(100, point); // limit rate to 300 Hz

    auto start = steady_clock::now();
    auto wait = milliseconds(10);
    auto dur = milliseconds(0);
    while (point > 0)
    {
        double dt = duration_cast<duration<double, 
               std::ratio<60>>>(wait).count();

        std::cout << dt << std::endl;
        point = rt.step(0, dt);
        std::cout << dur.count() << "\t" << point << std::endl;
        dur += wait;
        std::this_thread::sleep_until(start + dur);
    }
    return 0;
}
