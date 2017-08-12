#include <chrono>
#include <thread>
#include <cassert>
#include <inttypes.h>
#include <iostream>
#include <fstream>

#include <dtypes.h>
#include <control.h>

int main()
{
    namespace chrono = std::chrono;

    uav::State init = {0};

    // example configuration
    uav::Param prm = {20, 0, 0, {0, 0, 0.35, -1}, {0, 0, 1, -1},
                {1, 0, 1, -1}, {1, 0, 1, -1}, 0.3, 0.65, 250, 37};

    assert(!uav::Control::debug());

    uav::Control c(init, prm);
    if (c.align())
    {
        fprintf(stderr, "Something went wrong\n");
        return 1;
    }

    std::ofstream fout;
    fout.open("log/iters.txt", std::ios::out);

    const auto start = chrono::steady_clock::now();
    auto dur = chrono::milliseconds(0);
    const auto wait = chrono::milliseconds(1000/prm.freq);

    while (dur < chrono::seconds(30))
    {
        c.iterate();
        uav::State s = c.getstate();
        fout << tostring(s) << "\n";
        dur+=wait;
        std::this_thread::sleep_until(start + dur);
    }

    fout.flush();
    fout.close();

    return 0;
}
