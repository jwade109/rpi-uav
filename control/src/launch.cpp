#include <iostream>
#include <chrono>
#include <thread>
#include <cmath>
#include <signal.h>

#include <uav/logging>
#include <uav/hardware>
#include <uav/control>

bool cont = true;

void sigint(int signal)
{
    cont = false;
    uav::info << "Program interrupted.\n";
}

int main(int argc, char** argv)
{
    namespace chrono = std::chrono;

    signal(SIGINT, sigint);

    uav::param prm = {uav::param::f50hz, 1,
       {1,   0, 3,   INFINITY,
        10,  0, 20,  INFINITY,
        2,   0, 0,   INFINITY,
        6,   0, 4.5, INFINITY,
        6,   0, 4.5, INFINITY}};
    uav::state init{0};

    uav::controller c(prm);
    {
        uav::logstream paramlog("Param");
        uav::archive a;
        paramlog << (a << prm);
    }
    uav::logstream statelog("State");
    uav::logstream rawlog("Raw");

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

    uav::info << "Initializing...\n";
    std::cout << "Initializing..." << std::endl;
    if (sensors.begin() > 0)
    {
        std::cerr << "Sensor init failed." << std::endl;
        uav::error << "Sensor init failed.\n";
        uav::flush();
        return 1;
    }

    std::this_thread::sleep_for(std::chrono::seconds(2));

    auto start = chrono::steady_clock::now(), now = start;
    auto dt = chrono::milliseconds(1000/prm.freq), runtime = dt * 0;
    while ((now < start + chrono::seconds(5 * 60)) && cont)
    {
        while (now <= start + runtime)
            now = chrono::steady_clock::now();
        auto raw = sensors.get();
        c.step(raw);
        uav::state s = c.getstate();

        uav::archive sl, rl;
        rawlog << (rl << raw);
        statelog << (sl << s);

        std::cout << s.time[0]/1000.0 << " "
                  << s.position << " : "
                  << s.attitude[0] << " "
                  << s.attitude[1] << " "
                  << s.attitude[2] << "\r" << std::flush;

        now = chrono::steady_clock::now();
        runtime += dt;
    }

    if (cont) uav::info << "Program terminated normally.\n";
    uav::info << "Writing to file...\n";
    std::cout << std::endl << "Writing to file..." << std::endl;

    uav::flush();

    std::cout << "Done." << std::endl;

    return 0;
}
