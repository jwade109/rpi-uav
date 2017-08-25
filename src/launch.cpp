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
    uav::debug.push_back("Program interrupted.");
}

int main(int argc, char** argv)
{
    namespace chrono = std::chrono;

    signal(SIGINT, sigint);

    // initialize the controller
    uav::Param prm{uav::F100Hz, 0, 0, {0, 0, 1}, {0, 0, 0.3},
             {1, 0, 0.5}, {1, 0, 0.5}, 0.1, 0.65, 500, 41};
    uav::State init{0};

    bool debug = argc > 1 ? true : false;
    uav::Control c(init, prm, debug);

    // begin imu, bmp, and get home altitudes
    uav::debug.push_back("Aligning...");
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
    uint64_t mask = 0b11111000000001111111111;
    std::cout << uav::sheader(mask) << std::endl;

    auto start = chrono::steady_clock::now(), now = start;
    while ((now < start + chrono::seconds(100)) && cont)
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

    if (cont) uav::debug.push_back("Program terminated normally.");
    uav::debug.push_back("Writing to file...");
    std::cout << std::endl << "Writing to file..." << std::endl;

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
