#include <iostream>
#include <chrono>
#include <thread>
#include <cmath>
#include <cassert>
#include <string.h>
#include <stdbool.h>
#include <signal.h>

#include <ncurses.h>
#include <control.h>
#include <monitor.h>

bool cont = true;

void sigint(int signal)
{
    cont = false;
}

int main()
{
    namespace chrono = std::chrono;
    namespace log = uav::log;

    signal(SIGINT, sigint);

    // initialize the controller
    uav::Param prm{uav::F250Hz, 0, 0, {0, 0, 1}, {0, 0, 0.3},
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

    // write the new parameters to a file
    log::open();
    log::param = c.getparams();

    // print the params
    std::cout << uav::to_string(c.getparams()) << std::endl;

    uint64_t mask = 0b111110000000111111111;

    std::cout << uav::sheader(mask) << std::endl;

    auto start = chrono::steady_clock::now(), now = start;
    while ((now < start + chrono::seconds(1)) && cont)
    {
        // iterate the controller,
        // enable internal timing management
        c.iterate(true);
        uav::State s = c.getstate();
        log::states.put(s);

        // print the controller state
        std::cout << uav::to_string(s, mask);
        if (s.err)  std::cout << "!!!";
        else        std::cout << "   ";
        std::cout << "\r" << std::flush;

        now = chrono::steady_clock::now();
    }
    std::cout << "\nDone." << std::endl;

    // flush the log buffers
    log::flush();
    log::close();

    return 0;
}

/*
void draw(Drone d)
{
    Param params = d.getparams();
    Iter sc = d.getstate();

    int cols = 50, rows = 30;

    int s = cols/6;
    int cc[4] = {s, 2*s, 4*s, 5*s};
    int rc[4] = {3*rows/4, rows/2, rows/2, 3*rows/4};
    int motorbarh = rows - rc[0] - 2;
    int dcc = (cc[1] + cc[2])/2;
    int drc = (rc[0] + rc[1])/2;
    int draw[4] = {1, 0, 3, 2};

    for (int i = 0; i < rows; i++)
    {
        move(i, 0);
        clrtoeol();
    }

    mvprintw(0, 0, "Drone Interface (%dx%d)", cols, rows);
    mvprintw(2, 1, "Time: %.03lf", sc.t/1000.0);

    mvprintw(4, 1, "Heading:  %.2f", sc.h);
    mvprintw(5, 1, "Pitch:    %.2f", sc.p);
    mvprintw(6, 1, "Roll:     %.2f", sc.r);
    mvprintw(7, 1, "Calib:    %d %d %d %d",
        (sc.calib >> 6) % 4, (sc.calib >> 4) % 4,
        (sc.calib >> 2) % 2, sc.calib % 4);

    mvprintw(8, 1, "IMU Alt:  %.2f", sc.z1);
    mvprintw(9, 1, "RPi Alt:  %.2f %.2f", sc.z2, params.gz_wam);
    mvprintw(10,1, "Combined: %.2f %.2f", sc.dz, params.gz_lpf);

    mvprintw(2, 2*cols/3, "H PID: %.2f", sc.hov);
    mvprintw(3, 2*cols/3, "P PID: %.2f", sc.pov);
    mvprintw(4, 2*cols/3, "R PID: %.2f", sc.rov);
    mvprintw(5, 2*cols/3, "A PID: %.2f", sc.zov);

    mvprintw(rc[1] - 2, dcc, "^");
    mvprintw(drc - 2, dcc, "X");
    mvprintw(drc, dcc - 2, "Y-Z");

    mvprintw(1, cols/3, "   P    I    D");
    mvprintw(2, cols/3, "H: %.02lf %.02lf %.02lf",
            params.hpidg[0], params.hpidg[1], params.hpidg[2]);
    mvprintw(3, cols/3, "P: %.02lf %.02lf %.02lf",
            params.ppidg[0], params.ppidg[1], params.ppidg[2]);
    mvprintw(4, cols/3, "R: %.02lf %.02lf %.02lf",
            params.rpidg[0], params.rpidg[1], params.rpidg[2]);
    mvprintw(5, cols/3, "Z: %.02lf %.02lf %.02lf",
            params.zpidg[0], params.zpidg[1], params.zpidg[2]);

    for (int i = 0; i < 4; i++)
    {
        for (int j = rc[i] - motorbarh; j <= rc[i] + motorbarh; j++)
        {
            int jr = 100*(rc[i] - j)/motorbarh;
            if ((sc.motors[draw[i]] >= jr && jr >= 0) ||
                (sc.motors[draw[i]] <= jr && jr <= 0))
                mvprintw(j, cc[i], "#");
        }
        mvprintw(rc[i], cc[i] - 1, "M%d:%d", draw[i] + 1, sc.motors[draw[i]]);
    }
}
*/
