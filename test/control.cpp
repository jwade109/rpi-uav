#include <ncurses.h>
#include <stdio.h>
#include <math.h>
#include <string.h>

#include <TimeUtil.h>
#include <SerialIMU.h>
#include <PID.h>
#include <BMP085.h>

#define LOGF "log/log.txt"
#define NUM_CMDS 2

uint8_t freq = 20;
double home_alt_imu = 0;
double home_alt_bmp = 0;
uint64_t unix_time, old_time, offset;
int motors[4] = {0, 0, 0, 0};
int motor_max = 100;

SerialIMU imu;
BMP085    bmp;

PID heading_pid ( 0, 0, 0.30 );
PID pitch_pid   ( 1, 0, 0.5  );
PID roll_pid    ( 1, 0, 0.5  );
PID alt_pid     ( 0, 0, 1    );

char buffer[100] = {0};
uint8_t wptr = 0;

char cmd[NUM_CMDS][50] = {"quit", "git rekt"};

/*     
 *   0  X  3     0-indexed orientation of motors on the vehicle.
 *      |        motors[] stores the current power of each motor,
 *    Y-Z        draw[] stores the drawing order (L to R) onscreen.
 *
 *   1     2
 */

void draw(Message msg,
          double sec_alt,
          double filtered_alt,
          double hpid,
          double ppid,
          double rpid,
          double apid,
          double mg,
          char ch);

int parse(char ch);

int main(int argc, char** argv)
{    
    system("stty -F /dev/ttyACM0 115200");
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

    uint64_t file_time = getUnixTime(SEC);
    char filename[30];
    sprintf(filename, "log/%" PRIu64 ".txt", file_time);

    FILE* log = fopen(filename,"w");
    if (log == 0)
    {
        fprintf(stderr, "Failed to open log file\n");
        return 2;
    }
    fprintf(log, "Time,Heading,Pitch,Roll,RelAlt1,RelAlt2,AvgAlt,M1,M2,M3,M4,ch\n");

    if (argc > 1)
    {
        freq = strtol(argv[1], 0, 10);
    }

    offset = getUnixTime(MILLI) - imu.get().millis;
    home_alt_imu = imu.get().alt;
    home_alt_bmp = bmp.getAltitude();

    double old_alt = 0;
    old_time = 0;
   
    initscr();
    nodelay(stdscr, 1);
    nonl();

    for (;;)
    {
        Message msg = imu.get();
        float sec_alt = bmp.getAltitude();
        old_time = unix_time;
        unix_time = getUnixTime(MILLI);

        double alpha = 0.97;
        double average = (msg.alt + sec_alt - home_alt_imu - home_alt_bmp)/2;
        double filtered_alt = average * (1 - alpha) + old_alt * alpha;
        old_alt = filtered_alt;

        double hpid = heading_pid.seek(msg.heading, 150, 0.025);
        double ppid = pitch_pid.seek(msg.pitch, 0, 0.025);
        double rpid = roll_pid.seek(msg.roll, 0, 0.025);
        double apid = alt_pid.seek(filtered_alt, 0, 0.025);
        double mg = 35 / (cos(msg.pitch*M_PI/180)*cos(msg.roll*M_PI/180));

        motors[0] = /* std::max(0, std::min(100, (int) */ round(-hpid + -ppid +  rpid + apid + mg); // ));
        motors[1] = /* std::max(0, std::min(100, (int) */ round( hpid +  ppid +  rpid + apid + mg); // ));
        motors[2] = /* std::max(0, std::min(100, (int) */ round(-hpid +  ppid + -rpid + apid + mg); // ));
        motors[3] = /* std::max(0, std::min(100, (int) */ round( hpid + -ppid + -rpid + apid + mg); // )); 

        char ch = getch();
        if (ch == 3)
        {
            system("stty sane");
            return 130;
        }
        if (parse(ch) == 0)
        {
            system("stty sane");
            return 0;
        }

        fprintf(log, "%" PRIu64 " %.2lf %.2lf %.2lf %.2lf %.6lf %.6lf %.6lf %d %d %d %d %02x\n",
            unix_time, msg.heading, msg.pitch, msg.roll,
            msg.alt - home_alt_imu, sec_alt - home_alt_bmp,
            average, filtered_alt,
            motors[0], motors[1], motors[2], motors[3], ch);
        fflush(log);

        draw(msg, sec_alt, filtered_alt, hpid, ppid, rpid, apid, mg, ch);
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

void draw(Message msg,
          double sec_alt,
          double filtered_alt,
          double hpid,
          double ppid,
          double rpid,
          double apid,
          double mg,
          char ch)
{
    int rows, cols;
    getmaxyx(stdscr, rows, cols);
    
    int s = cols/6;
    int cc[4] = {s, 2*s, 4*s, 5*s};
    int rc[4] = {3*rows/4, rows/2, rows/2, 3*rows/4};
    int motor_bar_height = rows - rc[0] - 2;
    int dcc = (cc[1] + cc[2])/2;
    int drc = (rc[0] + rc[1])/2;
    int draw[4] = {1, 0, 3, 2};

    for (int i = 0; i < rows; i++)
    {
        move(i, 0);
        clrtoeol();
    }

    mvprintw(0, 0, "Drone Interface (%dx%d)", cols, rows);
    mvprintw(2, 1, "Time:     %.03lf", unix_time/1000.0);
    mvprintw(3, 1, "IMU Time: %.03lf %.03lf %.03lf",
        msg.millis/1000.0,
        (unix_time - offset)/1000.0,
        1000.0/(unix_time - old_time));
    mvprintw(4, 1, "Heading:  %.02lf", msg.heading);
    mvprintw(5, 1, "Pitch:    %.02lf", msg.pitch);
    mvprintw(6, 1, "Roll:     %.02lf", msg.roll);
    mvprintw(7, 1, "Calib:    %d %d %d %d",
        (msg.calib >> 6) % 4, (msg.calib >> 4) % 4,
        (msg.calib >> 2) % 2, msg.calib % 4);
    mvprintw(8, 1, "IMU Alt:  %.02lf", msg.alt);
    mvprintw(9, 1, "RPi Alt:  %.02f", sec_alt);
    mvprintw(10,1, "Combined: %.02f", filtered_alt);

    mvprintw(2, 2*cols/3, "H PID: %.02lf", hpid);
    mvprintw(3, 2*cols/3, "P PID: %.02lf", ppid);
    mvprintw(4, 2*cols/3, "R PID: %.02lf", rpid);
    mvprintw(5, 2*cols/3, "A PID: %.02lf", apid);
    mvprintw(5, 2*cols/3, "MG:    %.02lf", mg);

    mvprintw(rc[1] - 2, dcc, "^");
    mvprintw(drc - 2, dcc, "X");
    mvprintw(drc, dcc - 2, "Y-Z");

    mvprintw(1, cols/3, "   P    I    D");
    mvprintw(2, cols/3, "H: %.02lf %.02lf %.02lf",
        heading_pid.Kp, heading_pid.Ki, heading_pid.Kd);
    mvprintw(3, cols/3, "P: %.02lf %.02lf %.02lf",
        pitch_pid.Kp, pitch_pid.Ki, pitch_pid.Kd);
    mvprintw(4, cols/3, "R: %.02lf %.02lf %.02lf",
        roll_pid.Kp, roll_pid.Ki, roll_pid.Kd);
    mvprintw(5, cols/3, "A: %.02lf %.02lf %.02lf",
        alt_pid.Kp, alt_pid.Ki, alt_pid.Kd);
        
    for (int i = 0; i < 4; i++)
    {
        for (int j = rc[i] - motor_bar_height; j <= rc[i] + motor_bar_height; j++)
        {
            int jr = 100*(rc[i] - j)/motor_bar_height;
            if ((motors[draw[i]] >= jr && jr >= 0) ||
                (motors[draw[i]] <= jr && jr <= 0))
                mvprintw(j, cc[i], "#");
        }
        mvprintw(rc[i], cc[i] - 1, "M%d:%d", draw[i] + 1, motors[draw[i]]);
    }
    mvprintw(rows - 2, 1, "%c\t(%d)", ch, ch);
    mvprintw(rows - 1, 1, ":%s", buffer);
}

int parse(char ch)
{
    if (31 < ch && ch < 127 && wptr < 99)
    {
        buffer[wptr] = ch;
        wptr++;
    }
    else if (ch == 127 && wptr > 0)
    {
        wptr--;
        buffer[wptr] = 0;
    }
    else if (ch == 13)
    {
        int cmd_id = -1;
        for (int i = 0; i < NUM_CMDS; i++)
        {
            if (strcmp(buffer, cmd[i]) == 0)
            {
                cmd_id = i;
            }
        }
        memset(buffer, 0, 100);
        wptr = 0;
        return cmd_id;
    }
    return -1;
}
