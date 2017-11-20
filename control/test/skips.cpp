#include <iostream>
#include <iomanip>
#include <string>

#include <uav/logging>
#include <uav/control>

int main(int argc, char** argv)
{
    auto archives = argc > 1 ? uav::restore_sorted(argv[1]) :
                               uav::restore_sorted();

    if (archives.size() == 0)
    {
        std::cerr << "Invalid filename." << std::endl;
        return 1;
    }
    if (archives["Param"].size() == 0)
    {
        std::cerr << "No parameter info." << std::endl;
        return 2;
    }
    if (archives["Param"].size() > 1)
    {
        std::cerr << "Ambiguous: multiple parameter packets." << std::endl;
        return 3;
    }
    if (archives["State"].size() == 0)
    {
        std::cerr << "No state info." << std::endl;
        return 4;
    }

    uav::param prm;
    archives["Param"][0] >> prm;

    unsigned int dt = 1000/prm.freq;
    std::cerr << "Detected frequency of " << (int) prm.freq
              << " Hz (dt = " << dt << " ms)" << std::endl;
    
    unsigned int maxdt = 0, mindt = -1;
    unsigned long long ts = 0, prev, skips = 0;

    std::cout.setf(std::ios::fixed, std::ios::floatfield);
    std::cout.precision(3);

    uav::state s;
    bool first = true;
    for (uav::archive s_a : archives["State"])
    {
        s_a >> s;
        if (!first) prev = ts;
        ts = s.time[0];

        if (!first && ts - prev > maxdt) maxdt = ts - prev;
        if (!first && ts - prev < mindt) mindt = ts - prev;

        if (!first && ((ts - prev) != dt))
        {
            if (skips < 20)
            std::cout << std::setw(7) << prev/1000.0
                      << " -> " << std::setw(5) << ts/1000.0
                      << ": dt = " << ts - prev << std::endl;
            skips++;
        }
        first = false;
    }
    if (skips > 20) std::cout << "(Not showing " << skips - 20
        << " more timing issues)" << std::endl;

    std::cout << "Finished. " << skips << " timing errors over "
              << archives["State"].size() << " iterations, or "
              << ts/1000.0 << " seconds." << std::endl
              << "Max dt: " << maxdt
              << ", min dt: " << mindt << std::endl;

    return 0;
}
