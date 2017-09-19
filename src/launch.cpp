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
#include <motor.h>

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

    static_assert(uav::param::fields == 23, "Check yourself");
    uav::param prm = {uav::f125hz, 0, 0, {0, 0, 0.005, -1},
            {0, 0, 0.015, -1}, {0.1, 0, 0.02, -1}, {0.1, 0, 0.02, -1},
            0.1, 0.65, 500, 41};
    uav::state init{0};

    bool debug = argc > 1 ? true : false;
    uav::controller c(init, prm, debug);

    pwm_driver pwm;
    pwm.begin(0x40);
    pwm.reset();
    pwm.setPWMFreq(800);
    // uav::motor m1(pwm, 0), m2(pwm, 4), m3(pwm, 8), m4(pwm, 12);

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
    std::cout << uav::sheader(1 | (0b1111 << 20)) << std::endl;

    auto start = chrono::steady_clock::now(), now = start;
    while ((now < start + chrono::seconds(100)) && cont)
    {
        // iterate the controller,
        // enable internal timing management
        c.iterate(true);
        uav::state s = c.getstate();

        auto timer = chrono::steady_clock::now();
        for (int i = 0; i < 4; i++)
            pwm.setPin(i * 4, s.motors[i] * 40, false);
        chrono::duration<double, std::milli> mt =
            chrono::steady_clock::now() - timer;
        uav::debug(std::to_string(mt.count()));

        // print the controller state
        std::cout << uav::to_string(s, 1 | (0b1111 << 20));
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
