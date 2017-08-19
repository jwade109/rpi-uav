#include <iostream>
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
}

int main()
{
    namespace chrono = std::chrono;

    signal(SIGINT, sigint);

    // initialize the controller
    uav::Param prm{uav::F100Hz, 0, 0, {0, 0, 1}, {0, 0, 0.3},
             {1, 0, 0.5}, {1, 0, 0.5}, 0.1, 0.65, 500, 41};
    uav::State init{0};
    uav::Control c(init, prm);

    // begin imu, bmp, and get home altitudes
    std::cout << "Aligning..." << std::endl;
    if (c.align())
    {
        std::cout << "Failed to align!" << std::endl;
        return 1;
    }
    std::cout << "Alignment complete." << std::endl;

    const size_t maxlen = 1000*1000;
    std::deque<uav::State> list;

    // print the params
    std::cout << uav::to_string(c.getparams()) << std::endl;
    uint64_t mask = 0b111110000000111111111;
    std::cout << uav::sheader(mask) << std::endl;

    auto start = chrono::steady_clock::now(), now = start;
    while ((now < start + chrono::seconds(10)) && cont)
    {
        // iterate the controller,
        // enable internal timing management
        c.iterate(true);
        uav::State s = c.getstate();

        // print the controller state
        std::cout << uav::to_string(s, mask);
        if (s.err)  std::cout << "!!!";
        else        std::cout << "   ";
        std::cout << "\r" << std::flush;

        // add state to log, enforcing max size
        list.push_back(s);
        if (list.size() > maxlen)
            list.pop_front();

        now = chrono::steady_clock::now();
    }
    std::cout << "Writing to file..." << std::endl;

    namespace log = uav::log;
    std::ofstream data("log/data.bin", std::ios::out | std::ios::binary);
    char sbuf[uav::statelen], pbuf[uav::paramlen];
    uav::Param p = c.getparams();
    uav::to_buffer(p, pbuf);
    data.write(pbuf, uav::paramlen);
    while (!list.empty())
    {
        uav::to_buffer(list.front(), sbuf);
        list.pop_front();
        data.write(sbuf, uav::statelen);
    }

    if (!log::debug.empty())
    {
        std::ofstream debug("log/debug.txt", std::ios::out);
        while (!log::debug.empty())
        {
            debug << log::debug.front() << "\n";
            log::debug.pop_front();
        }
    }
    if (!log::warn.empty())
    {
        std::ofstream warn("log/warn.txt", std::ios::out);
        while (!log::warn.empty())
        {
            warn << log::warn.front() << "\n";
            log::warn.pop_front();
        }
    }
    if (!log::fatal.empty())
    {
        std::ofstream fatal("log/fatal.txt", std::ios::out);
        while (!log::fatal.empty())
        {
            fatal << log::fatal.front() << "\n";
            log::fatal.pop_front();
        }
    }
    std::cout << "Done." << std::endl;

    return 0;
}
