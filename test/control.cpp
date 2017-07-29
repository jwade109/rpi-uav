#include <ncurses.h>
#include <stdio.h>
#include <math.h>
#include <string.h>
#include <stdbool.h>

#include <TimeUtil.h>
#include <SerialIMU.h>
#include <PID.h>
#include <BMP085.h>

#define HDG_PID_P           0
#define HDG_PID_I           0
#define HDG_PID_D           0.3

#define PITCH_PID_P         1
#define PITCH_PID_I         0
#define PITCH_PID_D         0.5

#define ROLL_PID_P          1
#define ROLL_PID_I          0
#define ROLL_PID_D          0.5

#define ALT_PID_P           0
#define ALT_PID_I           0
#define ALT_PID_D           1

#define ALT_LPF_G           0.3
#define ALT_WAM_G           0.65

#define MOTOR_MAX           100
#define NUM_CMDS            3
#define CMD_SIZE            50

#define FREQUENCY           40;

typedef struct
{
    uint8_t freq;           // frequency of updates in hz
    double home_alt_imu;    // home point altitude from imu
    double home_alt_bmp;    // home point altitude from bmp085
    uint64_t imu_time_offset;   // epoch time - arduino time

    double hpidg[3];        // pid gains for yaw
    double ppidg[3];        // pid gains for pitch
    double rpidg[3];        // pid gains for roll
    double apidg[3];        // pid gains for altitude
    
    double g_lpf_alt;       // gain for alt low-pass filter
    double g_wam_alt;       // weighted average gain towards alta
}
params_t;

typedef struct
{
    uint64_t unix_time;     // epoch time in millis
    Message msg;            // message from arduino
    double altb;            // alt from external bmp085
    double dalt_lpf;        // wavg(alta,altb) - home, lpf

    double hpid;            // heading pid response
    double ppid;            // pitch pid response
    double rpid;            // roll pid response
    double apid;            // altitude pid response
    double mg;              // base motor thrust from F - mg = 0
    int16_t motors[4];      // stores thrust of each motor

    int keypress;           // keypress of this cycle
    char cmd[CMD_SIZE];     // command if there is one
}
step_t;

int rows, cols;
FILE* steplog;
FILE* paramlog;

params_t params;
step_t curr;
step_t prev;

SerialIMU imu;
BMP085    bmp;

PID heading_pid (HDG_PID_P,   HDG_PID_I,   HDG_PID_D  );
PID pitch_pid   (PITCH_PID_P, PITCH_PID_I, PITCH_PID_D);
PID roll_pid    (ROLL_PID_P,  ROLL_PID_I,  ROLL_PID_D );
PID alt_pid     (ALT_PID_P,   ALT_PID_I,   ALT_PID_D  );

char prompt[CMD_SIZE * 2] = {0};
char buffer[CMD_SIZE] = {0};
uint8_t wptr = 0;
char cmd[NUM_CMDS][CMD_SIZE] = {"quit", "pause", "unpause"};

bool ispaused;
int stepno;

int setup();
void iterate();
void draw();
void logstep();
void logparams();
void parse();

int main()
{
    int ret = setup();
    if (ret)
    {
        fprintf(stderr, "Something went wrong: %d\n", ret);
        return 1;
    }
    for (stepno = 0; stepno < 2000; stepno++)
    {
        if (!ispaused) iterate();
        parse();
        logstep();
        logparams();
        draw();
    }
    endwin();
}

int setup()
{
    ispaused = false;
    printf("Connecting to Arduino IMU...\n");
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

    steplog = fopen("log/steplog.txt","w");
    if (steplog == 0)
    {
        fprintf(stderr, "Failed to open step log file\n");
        return 2;
    }
    paramlog = fopen("log/paramlog.txt","w");
    if (paramlog == 0)
    {
        fprintf(stderr, "Failed to open parameter log file\n");
        return 3;
    }
    fprintf(steplog, "Time,Heading,Pitch,Roll,RelAlt1,RelAlt2,LPFAlt,M1,M2,M3,M4,ch,cmd\n");
    fprintf(paramlog, "Time,Freq,Offset,HomeAltA,HomeAltB,HP,HI,HD,PP,PI,PD,"
            "RP,RI,RD,AP,AI,AD,LPFGain,WAMGain\n");

    memset(&params, 0, sizeof(params_t));
    params.imu_time_offset = getUnixTime(MILLI) - imu.get().millis;
    params.home_alt_imu = imu.get().alt;
    params.home_alt_bmp = bmp.getAltitude();
    params.g_lpf_alt = ALT_LPF_G;
    params.g_wam_alt = ALT_WAM_G;
    params.freq = FREQUENCY;

    initscr();
    noecho();
    getmaxyx(stdscr, rows, cols);
    rows--;
    nodelay(stdscr, 1);
    keypad(stdscr, 1);

    memset(&curr, 0, sizeof(step_t));
    memset(&prev, 0, sizeof(step_t));

    return 0;
}

void iterate()
{
    uint64_t t = getUnixTime(MILLI) + 1;
    while(t % (1000/params.freq) != 0)
    {
        t++;
    }
    waitUntil(t, MILLI);
    
    memcpy(&prev, &curr, sizeof(step_t));
    memset(&curr, 0, sizeof(step_t));
    
    curr.msg = imu.get();
    curr.altb = bmp.getAltitude();
    curr.unix_time = getUnixTime(MILLI);

    double da = curr.msg.alt - params.home_alt_imu;
    double db = curr.altb - params.home_alt_bmp;
    double average = da * params.g_wam_alt + db * (1 - params.g_wam_alt);
    curr.dalt_lpf = average * (params.g_lpf_alt) +
                    prev.dalt_lpf * (1 - params.g_lpf_alt);
    curr.hpid = heading_pid.seek(curr.msg.heading, 150, 0.025);
    curr.ppid = pitch_pid.seek(curr.msg.pitch, 0, 0.025);
    curr.rpid = roll_pid.seek(curr.msg.roll, 0, 0.025);
    curr.apid = alt_pid.seek(curr.dalt_lpf, 0, 0.025);
    curr.mg = 35 / (cos(curr.msg.pitch * M_PI/180) * 
              cos(curr.msg.roll * M_PI/180));

    double resp[4];
    resp[0] = -curr.hpid - curr.ppid + curr.rpid + curr.apid + curr.mg;
    resp[1] =  curr.hpid + curr.ppid + curr.rpid + curr.apid + curr.mg;
    resp[2] = -curr.hpid + curr.ppid - curr.rpid + curr.apid + curr.mg;
    resp[3] =  curr.hpid - curr.ppid - curr.rpid + curr.apid + curr.mg;
        
    for (int i = 0; i < 4; i++)
        curr.motors[i] = std::max(0, std::min(100, (int) round(resp[i])));

    curr.keypress = getch();
}

void draw()
{    
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
    mvprintw(2, 1, "Time:     %.03lf", curr.unix_time/1000.0);
    mvprintw(3, 1, "IMU Time: %.03lf %.03lf %.03lf",
        curr.msg.millis/1000.0,
        (curr.unix_time - params.imu_time_offset)/1000.0,
        1000.0/(curr.unix_time - prev.unix_time));
    mvprintw(4, 1, "Heading:  %.02lf", curr.msg.heading);
    mvprintw(5, 1, "Pitch:    %.02lf", curr.msg.pitch);
    mvprintw(6, 1, "Roll:     %.02lf", curr.msg.roll);
    mvprintw(7, 1, "Calib:    %d %d %d %d",
        (curr.msg.calib >> 6) % 4, (curr.msg.calib >> 4) % 4,
        (curr.msg.calib >> 2) % 2, curr.msg.calib % 4);
    mvprintw(8, 1, "IMU Alt:  %.02lf", curr.msg.alt);
    mvprintw(9, 1, "RPi Alt:  %.02f", curr.altb);
    mvprintw(10,1, "Combined: %.02f", curr.dalt_lpf);

    mvprintw(2, 2*cols/3, "H PID: %.02lf", curr.hpid);
    mvprintw(3, 2*cols/3, "P PID: %.02lf", curr.ppid);
    mvprintw(4, 2*cols/3, "R PID: %.02lf", curr.rpid);
    mvprintw(5, 2*cols/3, "A PID: %.02lf", curr.apid);
    mvprintw(5, 2*cols/3, "MG:    %.02lf", curr.mg);

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
            if ((curr.motors[draw[i]] >= jr && jr >= 0) ||
                (curr.motors[draw[i]] <= jr && jr <= 0))
                mvprintw(j, cc[i], "#");
        }
        mvprintw(rc[i], cc[i] - 1, "M%d:%d", draw[i] + 1, curr.motors[draw[i]]);
    }
    mvprintw(rows - 2, 1, "%s", prompt);
    mvprintw(rows - 1, 1, ":%s", buffer);
    mvprintw(rows - 3, 1, "%d %s", stepno, curr.cmd);
}

void logstep()
{
    fprintf(steplog, "%" PRIu64 " %.2lf %.2lf %.2lf "
            "%.2lf %.5lf %.6lf "
            "%" PRId16 " %" PRId16 " %" PRId16 " %" PRId16 " "
            "%d \"%s\"\n",
            curr.unix_time, curr.msg.heading, curr.msg.pitch, curr.msg.roll,
            curr.msg.alt, curr.altb, curr.dalt_lpf,
            curr.motors[0], curr.motors[1], curr.motors[2], curr.motors[3],
            curr.keypress, curr.cmd);
    fflush(steplog);
}

void logparams()
{
    fprintf(paramlog, "%" PRIu64" %" PRIu8 " %" PRIu64 ""
            "%.02lf %.05lf "
            "%.2lf %.2lf %.2lf "
            "%.2lf %.2lf %.2lf "
            "%.2lf %.2lf %.2lf "
            "%.2lf %.2lf %.2lf "
            "%.2lf %.2lf\n",
            curr.unix_time, params.freq, params.imu_time_offset,
            params.home_alt_imu, params.home_alt_bmp,
            heading_pid.Kp, heading_pid.Ki, heading_pid.Kd,
            pitch_pid.Kp,   pitch_pid.Ki,   pitch_pid.Kd,
            roll_pid.Kp,    roll_pid.Ki,    roll_pid.Kd,
            alt_pid.Kp,     alt_pid.Ki,     alt_pid.Kd,
            params.g_lpf_alt, params.g_wam_alt);
}

void parse()
{
    int ch = curr.keypress;
    if (31 < ch && ch < 127 && wptr < CMD_SIZE - 1)
    {
        buffer[wptr] = ch;
        wptr++;
    }
    else if (ch == KEY_BACKSPACE && wptr > 0)
    {
        wptr--;
        buffer[wptr] = 0;
    }
    else if (ch == 10) // enter key
    {
        for (int i = 0; i < NUM_CMDS; i++)
        {
            if (strcmp(buffer, cmd[i]) == 0)
                sprintf(curr.cmd, cmd[i]);
        }
        memset(buffer, 0, CMD_SIZE);
        wptr = 0;
    }
    else sprintf(curr.cmd, "");
}
