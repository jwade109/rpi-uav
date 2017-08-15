#include <chrono>
#include <thread>
#include <cassert>
#include <inttypes.h>
#include <iostream>
#include <fstream>

#include <control.h>

int main()
{
    namespace chrono = std::chrono;

    uav::State init = {0};

    // example configuration
    uav::Param prm = {uav::F20Hz, 0, 0, {0, 0, 0.35, -1}, {0, 0, 1, -1},
                {1, 0, 1, -1}, {1, 0, 1, -1}, 0.3, 0.65, 250, 37};

    std::cout << uav::sheader << std::endl << uav::pheader << std::endl;

    assert(!uav::Control::debug());

    uav::Control c(init, prm);
    if (c.align())
    {
        fprintf(stderr, "Something went wrong\n");
        return 1;
    }

    auto start = chrono::steady_clock::now();
    auto maxdt = chrono::nanoseconds(0);
    unsigned long long i = 0;
    while (i < 1000000)
    {
        auto t1 = chrono::steady_clock::now();
        c.iterate(false);
        i++;
        chrono::nanoseconds dt = chrono::steady_clock::now() - t1;
        if (dt > maxdt)
            maxdt = dt;
    }
    auto runtime = chrono::steady_clock::now() - start;

    std::cout << i << " iterations of Control::iterate() took "
              << runtime.count()/1000000.0 << " ms ("
              << 1.0 * runtime.count()/i
              << " ns avg), with max dt equal to "
              << maxdt.count()/1000000.0 << " ms" << std::endl;

    return 0;
}
