#include <stdio.h>
#include <iostream>
#include <fstream>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include <PID.h>
#include <TimeUtil.h>
#include <ncurses.h>

using namespace std;
typedef double Motor;

int main(int argc, char** argv)
{
    initscr();
    refresh();

    double P = 8, I = 0, D = 3, dt = 0.05;
    PID control(P, I, D, -1);
    double omega = 0;
    int count = 0;
    int leverArm = 1;
    double setpoint = 0;
    double theta = -60;
    int armLength = 1;

    waitFor(2, SEC);
    refresh();

    mvprintw(0, 0, "Kd: %.2f\tKi: %.2f\tKd: %.2f\tSetpoint: %.2f\n",
        control.Kp, control.Ki, control.Kd, setpoint);

    uint64_t start_time = getUnixTime(MICRO);

    for (int iter = 0; iter < 60/dt; iter++)
    {
        double t = iter * dt;
        Motor motorR, motorL;
        motorL = control.seek(theta, setpoint, dt);
        motorR = -motorL;
        double alpha = armLength * (motorL - motorR);
        omega += alpha * dt;
        theta += omega * dt;
        move(2, 1);
        clrtoeol();
        printw("T:   \t%lf", t);
        move(3, 1);
        clrtoeol();
        printw("Mtrs:\t%lf\t%lf", motorL, motorR);
        move(4, 1);
        clrtoeol();
        printw("Roll:\t%lf", theta);
        move(6, 1);
        clrtoeol();
        printw("PID: \t%lf\t%lf\t%lf",
            control.p_response, control.i_response, control.d_response);
        move(8, 1);
        clrtoeol();
        for (int i = -45; i <= 45; i++)
        {
            int pos = (int) (theta/2);
            if (i == 0)
                printw("*");
            else if (i > pos && i < 0 || i < pos && i > 0)
                printw("|");
            else
                printw(" ");
        }
        refresh();
        uint64_t time = start_time + (t + dt) * SEC;
        waitUntil(time, MICRO);
        count++;
    }
    endwin();
    return 0;
}
