#include <iostream>
#include <iomanip>
#include <chrono>
#include <thread>
#include <cmath>
#include <cassert>
#include <string.h>
#include <stdbool.h>
#include <signal.h>

#include <ncurses.h>
#include <control.h>

bool cont = true;

void sigint(int signal)
{
    cont = false;
    uav::info("Program interrupted.");
}

int main(int argc, char** argv)
{
    namespace chrono = std::chrono;

    signal(SIGINT, sigint);

    static_assert(uav::param_fields == 23, "Check yourself before you segfault"); 
    uav::param prm = {uav::f250hz, 0, 0, {0, 0, 0.005, -1},
            {0, 0, 0.015, -1}, {0.1, 0, 0.02, -1}, {0.1, 0, 0.02, -1},
            0.1, 0.65, 500, 41};
    uav::state init{0};

    bool debug = argc > 1 ? true : false;
    uav::Control c(init, prm, debug);

    // begin imu, bmp, and get home altitudes
    uav::info("Aligning...");
    std::cout << "Aligning..." << std::endl;
    if (c.align())
    {
        std::cout << "Failed to align!" << std::endl;
        uav::flush();
        return 1;
    }
    std::cout << "Alignment complete." << std::endl;

    uav::include(c.getparams());

    // print the params
    std::cout << uav::pheader() << std::endl;
    std::cout << uav::to_string(c.getparams()) << std::endl;
    std::cout << uav::sheader(1) << std::endl;

    auto start = chrono::steady_clock::now(), now = start;
    while ((now < start + chrono::seconds(100)) && cont)
    {
        // iterate the controller,
        // enable internal timing management
        c.iterate(true);
        uav::state s = c.getstate();

        // print the controller state
        std::cout << uav::to_string(s, 1);
        if (s.err)  std::cout << " !";
        else        std::cout << "  ";
        std::cout << "\r" << std::flush;

        // add state to log
        uav::include(s);
        now = chrono::steady_clock::now();
    }

    if (cont) uav::info("Program terminated normally.");
    uav::info("Writing to file...");
    std::cout << std::endl << "Writing to file..." << std::endl;

    uav::flush();

    std::cout << "Done." << std::endl;

    return 0;
}
