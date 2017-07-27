#include <ncurses.h>
#include <stdio.h>
#include <math.h>
#include <string.h>

#include <TimeUtil.h>
#include <SerialIMU.h>
#include <PID.h>
#include <BMP085.h>

using namespace std;

int main(int argc, char** argv)
{    
    SerialIMU imu;
    BMP085 bmp;
    int imu_start = imu.begin();
    int bmp_start = bmp.begin();

    if (imu_start | bmp_start)
    {
        fprintf(stderr, "Failure to begin sensor: IMU (%d), BMP085 (%d)\n",
            imu_start, bmp_start);
        return 1;
    }

    printf("IMU initializing... (3 seconds)\n");
    waitFor(3, SEC);

    FILE* log;
    log = fopen("log.txt","w");
    if (log == 0)
    {
        fprintf(stderr, "Failed to open log file\n");
        return 2;
    }
 
    uint8_t freq = 20; // hz
    if (argc > 1)
    {
        freq = strtol(argv[1], 0, 10);
    }
    uint64_t offset = getUnixTime(MILLI) - imu.get().millis;
    double home_alt_imu = imu.get().alt;
    double home_alt_bmp = bmp.getAltitude();
    double old_alt = 0;
    double old_time = 0;

    double gains[4][3] = {{0,    0, 0.30},
                          {1,    0, 0.5 },
                          {1,    0, 0.5 },
                          {0,    0, 1   }};

    PID heading_pid(gains[0][0], gains[0][1], gains[0][2]);
    PID pitch_pid(  gains[1][0], gains[1][1], gains[1][2]);
    PID roll_pid(   gains[2][0], gains[2][1], gains[2][2]);
    PID alt_pid(    gains[3][0], gains[3][1], gains[3][2]);

    char buffer[100];
    memset(buffer, 0, 100);
    int count = 0;

    int rows, cols;
    initscr();
    getmaxyx(stdscr, rows, cols);
    int m[4], map[4] = {1, 0, 3, 2};

    nodelay(stdscr, 1);  

    for (;;)
    {
        Message msg = imu.get();
        float sec_alt = bmp.getAltitude();

        getmaxyx(stdscr, rows, cols); 
        int s = cols/6;
        int cc[4] = {s, 2*s, 4*s, 5*s};
        int rc[4] = {3*rows/4, rows/2, rows/2, 3*rows/4};
        int height = rows - rc[0] - 2;
        for (int i = 0; i < rows; i++)
        {
            move(i, 0);
            clrtoeol();
        }

        mvprintw(0, 0, "Drone Interface (%dx%d)(%d, %d)",
            cols, rows, imu_start, bmp_start);

        uint64_t time = getUnixTime(MILLI);
        double millis = time/1000.0;


        mvprintw(2, 1, "Time:     %.03lf", getUnixTime(MILLI)/1000.0);
        mvprintw(3, 1, "IMU Time: %.03lf %.03lf %.03lf",
            msg.millis/1000.0, (getUnixTime(MILLI) - offset)/1000.0,
            1.0/(millis - old_time));
        mvprintw(4, 1, "Heading:  %.02lf", msg.heading);
        mvprintw(5, 1, "Pitch:    %.02lf", msg.pitch);
        mvprintw(6, 1, "Roll:     %.02lf", msg.roll);
        mvprintw(7, 1, "Calib:    %d %d %d %d",
            (msg.calib >> 6) % 4, (msg.calib >> 4) % 4,
            (msg.calib >> 2) % 2, msg.calib % 4);
        mvprintw(8, 1, "IMU Alt:  %.02lf", msg.alt);
        mvprintw(9, 1, "RPi Alt:  %.02f", sec_alt);

        old_time = millis;

        double alpha = 0.97;
        double new_alt = (msg.alt + sec_alt - home_alt_imu - home_alt_bmp)/2;
        double filtered_alt = new_alt * (1 - alpha) + old_alt * alpha;
        old_alt = filtered_alt;

        mvprintw(10,1, "Combined: %.02f", filtered_alt);

        double hpid = heading_pid.seek(msg.heading, 150, 0.025);
        double ppid = pitch_pid.seek(msg.pitch, 0, 0.025);
        double rpid = roll_pid.seek(msg.roll, 0, 0.025);
        double apid = alt_pid.seek(filtered_alt, 0, 0.025);
        double mg = 35 / (cos(msg.pitch*M_PI/180)*cos(msg.roll*M_PI/180));

        mvprintw(2, 2*cols/3, "H PID: %.02lf", hpid);
        mvprintw(3, 2*cols/3, "P PID: %.02lf", ppid);
        mvprintw(4, 2*cols/3, "R PID: %.02lf", rpid);
        mvprintw(5, 2*cols/3, "A PID: %.02lf", apid);
        mvprintw(5, 2*cols/3, "MG:    %.02lf", mg);

        int dcc = (cc[1] + cc[2])/2;
        int drc = (rc[0] + rc[1])/2;

        mvprintw(rc[1] - 2, dcc, "^");
        mvprintw(drc - 2, dcc, "X");
        mvprintw(drc - 1, dcc, "|");
        mvprintw(drc, dcc - 2, "Y-Z");

        m[0] = -hpid + -ppid +  rpid + apid + mg;
        m[1] =  hpid +  ppid +  rpid + apid + mg;
        m[2] = -hpid +  ppid + -rpid + apid + mg;
        m[3] =  hpid + -ppid + -rpid + apid + mg;

        char labels[4] = {'H', 'P', 'R', 'A'};
        for (int i = 0; i < 4; i++)
        {
            mvprintw(2 + i, cols/3, "%c: ", labels[i]);
            for (int j = 0; j < 3; j++)
            {
                printw("%.02lf ", gains[i][j]);
            }
        }

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

        fprintf(log, "%" PRIu64 "\t%lf\t%lf\t%lf\t%lf\n",
            time, msg.alt - home_alt_imu, sec_alt - home_alt_bmp,
            new_alt, filtered_alt);
        fflush(log);

        char ch = getch();
        mvprintw(rows - 1, 1, ":%c (%d)", ch, ch);

        refresh();

        uint64_t curr = getUnixTime(MILLI) + 1;
        while(curr % (1000/freq) != 0)
        {
            curr++;
        }
        waitUntil(curr, MILLI);
    }
    endwin();

    return 0;
}
