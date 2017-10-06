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
#include <pwm.h>

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

    uav::param prm = {uav::f50hz, 0, 0,
        {1,   0, 3,   0},
        {10,  0, 20,   3},
        {2,   0, 0,   -1},
        {6,   0, 4.5, -1},
        {6,   0, 4.5, -1},
        0.1, 0.65, 8, 40, 9.81/4};
    uav::state init{0};

    bool debug = argc > 1 ? true : false;
    uav::controller c(init, prm, debug);

    pwm_driver pwm;
    pwm.begin(0x40);
    pwm.reset();
    pwm.setPWMFreq(800);

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
    namespace fmt = uav::fmt;
    auto format = fmt::time | fmt::configuration | fmt::status;

    std::cout << uav::param::header() << std::endl;
    std::cout << uav::to_string(c.getparams()) << std::endl;
    std::cout << uav::state::header(format) << std::endl;

    auto start = chrono::steady_clock::now(), now = start;
    while ((now < start + chrono::seconds(5 * 60)) && cont)
    {
        // iterate the controller,
        // enable internal timing management
        c.iterate(true);
        uav::state s = c.getstate();

        for (int i = 0; i < 4; i++)
            pwm.setPin(i * 4, s.motors[i] * 40, false);

        // print the controller state
        std::cout << uav::to_string(s, format);
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
