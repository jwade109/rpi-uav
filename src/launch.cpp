#include <iostream>
#include <chrono>
#include <thread>
#include <cmath>
#include <signal.h>

#include <uavcore.h>
#include <sensors.h>
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

    uav::param prm = {uav::f50hz,
        {1,   0, 3,   0},
        {10,  0, 20,   3},
        {2,   0, 0,   -1},
        {6,   0, 4.5, -1},
        {6,   0, 4.5, -1},
        8, 40, 9.81/4};
    uav::state init{0};

    uav::controller c(init, prm);

    /*
    pwm_driver pwm;
    if (pwm.begin(0x40) > 0)
    {
        std::cerr << "PWM init failed." << std::endl;
        uav::error << "PWM init failed." << std::endl;
        uav::flush();
        return 1;
    }
    pwm.reset();
    pwm.setPWMFreq(800);
    */

    uav::sensor_hub sensors;

    uav::info("Initializing...");
    std::cout << "Initializing..." << std::endl;
    if (sensors.begin() > 0)
    {
        std::cerr << "Sensor init failed." << std::endl;
        uav::error("Sensor init failed.");
        uav::flush();
        return 1;
    }

    uav::include(c.getparams());
    auto format = uav::fmt::time | uav::fmt::config;

    std::cout << uav::param::header() << std::endl;
    std::cout << uav::to_string(c.getparams()) << std::endl;
    std::cout << uav::state::header(format) << std::endl;

    auto start = chrono::steady_clock::now(), now = start;
    while ((now < start + chrono::seconds(5 * 60)) && cont)
    {
        c.step(sensors.get());
        uav::state s = c.getstate();

        // for (int i = 0; i < 4; i++)
        //    pwm.setPin(i * 4, s.motors[i] * 40, false);

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
