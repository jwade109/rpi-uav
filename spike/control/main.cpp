#include <ncurses.h>
#include <stdio.h>
#include <math.h>

#include <TimeUtil.h>
#include <SerialIMU.h>
#include <PID.h>

int main()
{
    SerialIMU imu;
    int start = imu.begin();

    PID heading_pid(1, 0, 0.1, -1);
    PID pitch_pid(1, 0, 0.1, -1);
    PID roll_pid(1, 0, 0.1, -1);
    PID alt_pid(1, 0, 0.1, -1);

    int rows, cols;
    initscr();
    getmaxyx(stdscr, rows, cols);
    int m[4], map[4] = {1, 0, 3, 2};

    for (;;)
    {
        getmaxyx(stdscr, rows, cols);
        int s = cols/8 - 1;
        int cc[4] = {s, 2*s, 4*s, 5*s};
        int rc[4] = {3*rows/4, rows/2, rows/2, 3*rows/4};
        int height = rows - rc[0] - 2;
        for (int i = 0; i < rows; i++)
        {
            move(i, 0);
            clrtoeol();
        }

        mvprintw(0, 0, "Drone Interface (%dx%d)(%d)", cols, rows, start);
        Message msg = imu.get();

        mvprintw(2, 1, "Time:     %.03lf", getUnixTime(MILLI)/1000.0);
        mvprintw(3, 1, "IMU Time: %.03lf", msg.millis/1000.0);
        mvprintw(4, 1, "Heading:  %.02lf", msg.heading);
        mvprintw(5, 1, "Pitch:    %.02lf", msg.pitch);
        mvprintw(6, 1, "Roll:     %.02lf", msg.roll);
        mvprintw(7, 1, "Calib:    %d %d %d %d",
            (msg.calib >> 6) % 4, (msg.calib >> 4) % 4,
            (msg.calib >> 2) % 2, msg.calib % 4);
        mvprintw(8, 1, "Alt:      %.02lf", msg.alt);

        double hpid = heading_pid.seek(msg.heading, 0, 0.025);
        double ppid = pitch_pid.seek(msg.pitch, 0, 0.025);
        double rpid = roll_pid.seek(msg.roll, 0, 0.025);
        double apid = alt_pid.seek(msg.alt, 0, 0.025);

        mvprintw(2, cols/2, "H PID: %.02lf", hpid);
        mvprintw(3, cols/2, "P PID: %.02lf", ppid);
        mvprintw(4, cols/2, "R PID: %.02lf", rpid);
        mvprintw(5, cols/2, "A PID: %.02lf", apid);

        int dcc = (cc[1] + cc[2])/2;
        int drc = (rc[0] + rc[1])/2;

        mvprintw(rc[1] - 2, dcc, "^");
        mvprintw(drc - 2, dcc, "X");
        mvprintw(drc - 1, dcc, "|");
        mvprintw(drc, dcc - 2, "Y-Z");

        double hk = 0.35, pk = 1, rk = 1, ak = 0;
        m[0] = -hk * hpid + -pk * ppid + rk * rpid + ak * apid;
        m[1] = hk * hpid + pk * ppid + rk * rpid + ak * apid;
        m[2] = -hk * hpid + pk * ppid + -rk * rpid + ak * apid;
        m[3] = hk * hpid + -pk * ppid + -rk * rpid + ak * apid;

        for (int i = 0; i < 4; i++)
        {
            for (int j = rc[i] - height; j <= rc[i] + height; j++)
            {
                int jr = 100*(rc[i] - j)/height;
                if ((m[map[i]] >= jr && jr >= 0) ||
                    (m[map[i]] <= jr && jr <= 0))
                    mvprintw(j, cc[i], "#");
            }
            mvprintw(rc[i], cc[i] - 1, "M%d:%d", map[i] + 1, m[map[i]]);
        }

        refresh();
        waitFor(25, MILLI);
    }
    endwin();

    return 0;
}
