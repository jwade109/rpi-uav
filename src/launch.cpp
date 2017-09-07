#include <iostream>
#include <iomanip>
#include <chrono>
#include <thread>
#include <cmath>
#include <cassert>
#include <string.h>
#include <stdbool.h>
#include <signal.h>
#include <deque>

#include <ncurses.h>
#include <control.h>

bool cont = true;

void sigint(int signal)
{
    cont = false;
    uav::debug_write("Program interrupted.");
}

int main(int argc, char** argv)
{
    namespace chrono = std::chrono;

    signal(SIGINT, sigint);

    static_assert(uav::param_fields == 23, "Check yourself before you segfault"); 
    uav::param prm = {uav::f50hz, 0, 0, {0, 0, 0.005, -1},
            {0, 0, 0.015, -1}, {0.1, 0, 0.02, -1}, {0.1, 0, 0.02, -1},
            0.1, 0.65, 500, 41};
    uav::state init{0};

    bool debug = argc > 1 ? true : false;
    uav::Control c(init, prm, debug);

    // begin imu, bmp, and get home altitudes
    uav::debug_write("Aligning...");
    std::cout << "Aligning..." << std::endl;
    if (c.align())
    {
        std::cout << "Failed to align!" << std::endl;
        return 1;
    }
    std::cout << "Alignment complete." << std::endl;

    std::deque<uav::state> list;

    // print the params
    std::cout << uav::pheader() << std::endl;
    std::cout << uav::to_string(c.getparams()) << std::endl;
    uint64_t mask = 0b1000000000000000000000001;
    std::cout << uav::sheader(mask) << std::endl;

    auto start = chrono::steady_clock::now(), now = start;
    while ((now < start + chrono::seconds(100)) && cont)
    {
        // iterate the controller,
        // enable internal timing management
        c.iterate(true);
        uav::state s = c.getstate();

        // print the controller state
        std::cout << uav::to_string(s, mask);
        if (s.err)  std::cout << " !";
        else        std::cout << "  ";
        std::cout << "\r" << std::flush;

        // add state to log
        list.push_back(s);
        now = chrono::steady_clock::now();
    }

    if (cont) uav::debug_write("Program terminated normally.");
    uav::debug_write("Writing to file...");
    std::cout << std::endl << "Writing to file..." << std::endl;

    std::ofstream data("log/data.bin", std::ios::out | std::ios::binary);
    uav::param p = c.getparams();
    auto pbin = uav::to_binary(p);
    data.write((const char*) pbin.data(), uav::param_size);
    while (!list.empty())
    {
        auto b = uav::to_binary(list.front());
        list.pop_front();
        data.write((const char*) b.begin(), uav::state_size);
    }

    if (!uav::debug.empty())
    {
        std::ofstream debug("log/debug.txt", std::ios::out);
        while (!uav::debug.empty())
        {
            debug << uav::debug.front() << "\n";
            uav::debug.pop_front();
        }
    }
    if (!uav::info.empty())
    {
        std::ofstream info("log/info.txt", std::ios::out);
        while (!uav::info.empty())
        {
            info << uav::info.front() << "\n";
            uav::info.pop_front();
        }
    }
    if (!uav::error.empty())
    {
        std::ofstream error("log/error.txt", std::ios::out);
        while (!uav::error.empty())
        {
            error << uav::error.front() << "\n";
            uav::error.pop_front();
        }
    }
    std::cout << "Done." << std::endl;

    return 0;
}
