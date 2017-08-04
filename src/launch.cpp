#include <ncurses.h>
#include <stdio.h>
#include <math.h>
#include <string.h>
#include <stdbool.h>

#include <timeutil.h>
#include <ardimu.h>
#include <pid.h>
#include <bmp.h>
#include <filebuffer.h>

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
#define NUM_CMDS            4
#define CMD_SIZE            50

#define FREQUENCY           50
#define DRAW_FREQ           20

// sizeof(params_t) = 1 + 2*4 + 4*4*4 + 2*4 = 81 bytes

typedef struct
{
    uint8_t freq;           // frequency of updates in hz
    float z1h;              // home point altitude from imu
    float z2h;              // home point altitude from bmp085

    float apidg[4];         // pid gains for altitude
    float hpidg[4];         // pid gains for yaw
    float ppidg[4];         // pid gains for pitch
    float rpidg[4];         // pid gains for roll
    
    float gz_lpf;           // gain for alt low-pass filter
    float gz_wam;           // weighted average gain towards alta
    float mg;               // weight of vehicle as percent of max thrust
}
params_t;

// sizeof(step_t) = 8 + 11*4 + 4 + 1 = 57 bytes

typedef struct
{
    uint64_t t;                 // epoch time in millis
    
    float z1;                   // alt from arduino imu
    float z2;                   // alt from external bmp085
    float dz;                   // filtered altitude from home point

    float h, p, r;              // heading, pitch, roll
    uint8_t calib;
    
    float zov, hov, pov, rov;   // respective pid response
    
    uint8_t motors[4];          // stores thrust of each motor
}
step_t;

int rows, cols;
FileBuffer steplog("log/steplog.txt");
FILE* paramlog;
FILE* events;

params_t params;
step_t sc;
step_t sp;

Arduino imu;
BMP085  bmp;

PID hpid(HDG_PID_P,   HDG_PID_I,   HDG_PID_D  );
PID ppid(PITCH_PID_P, PITCH_PID_I, PITCH_PID_D);
PID rpid(ROLL_PID_P,  ROLL_PID_I,  ROLL_PID_D );
PID zpid(ALT_PID_P,   ALT_PID_I,   ALT_PID_D  );

int stepno;

int setup();
void iterate();
void draw();
void logstep();
void logstepbin();
void logparams();

double trim(double val, double min, double max);

int main()
{
    int ret = setup();
    if (ret)
    {
        fprintf(stderr, "Something went wrong: %d\n", ret);
        return 1;
    }
    stepno = 0;

    while (stepno < 3)
    {
        uint64_t t = unixtime(milli) + 1;
        while (t % (1000/params.freq) > 0) t++;
        waituntil(t, milli);
  
        iterate();
        logstepbin();
        
        if (stepno % (params.freq/20) == 0)
            draw();

        stepno++;
    }
    fprintf(events, "[%" PRIu64 "] ", unixtime(milli));
    fprintf(events, "Stepno limit reached, terminating\n");
    fflush(events);
    steplog.flush();
    endwin();

    return 0;
}

int setup()
{
    timer();

    printf("Connecting to Arduino IMU...\n");
    int imu_start = imu.begin();
    int bmp_start = bmp.begin();

    if (imu_start | bmp_start)
    {
        fprintf(stderr, "Failure to begin sensor: IMU (%d), BMP085 (%d)\n",
            imu_start, bmp_start);
        return 1;
    }

    printf("IMU initializing... (3 seconds)\n");
    for (int i = 0; i < 75; i++)
    {
        printf("[");
        for (int j = 0; j < i; j++)
            printf("|");
        for (int j = i; j < 74; j++)
            printf(" ");
        printf("]");
        fflush(stdout);
        printf("\r");
        waitfor(3000/75);
    }
    fflush(stdout);
    waitfor(100);

    steplog.begin();
    paramlog = fopen("log/paramlog.txt","w");
    if (paramlog == 0)
    {
        fprintf(stderr, "Failed to open parameter log file\n");
        return 3;
    }
    events = fopen("log/events.txt","w");
    if (events == 0)
    {
        fprintf(stderr, "Failed to open events log file\n");
        return 4;
    }
    steplog.push("Time Heading Pitch Roll RelAlt1 RelAlt2 LPFAlt M1 M2 M3 M4 Key\n");
    fprintf(paramlog, "Time Freq Offset HomeAltA HomeAltB HP HI HD PP PI PD "
            "RP RI RD AP AI AD LPFGain WAMGain Cmd\n");

    memset(&params, 0, sizeof(params_t));
    
    params.z1h = imu.get().alt;
    params.z2h = bmp.getAltitude();
    params.gz_lpf = ALT_LPF_G;
    params.gz_wam = ALT_WAM_G;
    params.freq = FREQUENCY;

    initscr();
    noecho();
    getmaxyx(stdscr, rows, cols);
    rows--;
    nodelay(stdscr, 1);
    keypad(stdscr, 1);

    memset(&sc, 0, sizeof(step_t));
    memset(&sc, 0, sizeof(step_t));

    uint64_t dt = timer(micro);
    if (dt > 1000)
    {
        fprintf(events, "[%" PRIu64 "] ", unixtime(milli));
        fprintf(events, "Setup: %" PRIu64 " us\n", dt);
    }
    fflush(events);

    return 0;
}

void iterate()
{
    timer();

    memcpy(&sp, &sc, sizeof(step_t));
    memset(&sc, 0, sizeof(step_t));
    
    sc.t = unixtime(milli);
    imu.get(sc.h, sc.r, sc.p, sc.z1, sc.calib);
    sc.z2 = bmp.getAltitude();

    double dz1 = sc.z1 - params.z1h;
    double dz2 = sc.z2 - params.z2h;
    double zavg = dz1 * params.gz_wam + dz2 * (1 - params.gz_wam);
    sc.dz = zavg * (params.gz_lpf) + sp.dz * (1 - params.gz_lpf);

    sc.hov = hpid.seek(sc.h,  150, 0.025);
    sc.pov = ppid.seek(sc.p,  0,   0.025);
    sc.rov = rpid.seek(sc.r,  0,   0.025);
    sc.zov = zpid.seek(sc.dz, 0,   0.025);

    double hover = params.mg / (cos(sc.p * M_PI/180) * cos(sc.r * M_PI/180));

    double resp[4];
    resp[0] = -sc.hov - sc.pov + sc.rov + sc.zov + hover;
    resp[1] =  sc.hov + sc.pov + sc.rov + sc.zov + hover;
    resp[2] = -sc.hov + sc.pov - sc.rov + sc.zov + hover;
    resp[3] =  sc.hov - sc.pov - sc.rov + sc.zov + hover;
        
    for (int i = 0; i < 4; i++)
        sc.motors[i] = trim(resp[i], 0, 100);

    uint64_t dt = sc.t - sp.t;
    if (dt > 1000/params.freq && sp.t > 0)
    {
        fprintf(events, "[%" PRIu64 "] ", unixtime(milli));
        fprintf(events, "Skips: dt = %d\n", dt);
        fflush(events);
    }

    dt = timer(micro);
    if (dt > 1000)
    {
        fprintf(events, "[%" PRIu64 "] ", unixtime(milli));
        fprintf(events, "Iter: %" PRIu64 " us\n", dt);
    }
    fflush(events);
}

void draw()
{
    timer();

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
    mvprintw(2, 1, "Time: %.03lf %.03lf %d",
        sc.t/1000.0, 1000.0/(sc.t - sp.t), stepno);
    
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
    mvprintw(2, cols/3, "H: %.02lf %.02lf %.02lf", hpid.Kp, hpid.Ki, hpid.Kd);
    mvprintw(3, cols/3, "P: %.02lf %.02lf %.02lf", ppid.Kp, ppid.Ki, ppid.Kd);
    mvprintw(4, cols/3, "R: %.02lf %.02lf %.02lf", rpid.Kp, rpid.Ki, rpid.Kd);
    mvprintw(5, cols/3, "A: %.02lf %.02lf %.02lf", zpid.Kp, zpid.Ki, zpid.Kd);
        
    for (int i = 0; i < 4; i++)
    {
        for (int j = rc[i] - motor_bar_height; j <= rc[i] + motor_bar_height; j++)
        {
            int jr = 100*(rc[i] - j)/motor_bar_height;
            if ((sc.motors[draw[i]] >= jr && jr >= 0) ||
                (sc.motors[draw[i]] <= jr && jr <= 0))
                mvprintw(j, cc[i], "#");
        }
        mvprintw(rc[i], cc[i] - 1, "M%d:%d", draw[i] + 1, sc.motors[draw[i]]);
    }

    uint64_t dt = timer(micro);
    if (dt > 1000)
    {
        fprintf(events, "[%" PRIu64 "] ", unixtime(milli));
        fprintf(events, "Draw: %" PRIu64 " us\n", dt);
    }
    fflush(events);
}

void logstep()
{
    timer();

    char str[200];
    sprintf(str, "%" PRIu64 " %8.2lf %8.2lf %8.2lf "
            "%7.2lf %10.5lf %11.6lf "
            "%3d %3d %3d %3d %4d\n",
            sc.t, sc.h, sc.p, sc.r, sc.z1, sc.z2, sc.dz,
            (int) sc.motors[0], (int) sc.motors[1],
            (int) sc.motors[2], (int) sc.motors[3]);
    steplog.push(str);

    uint64_t dt = timer(micro);
    if (dt > 1000)
    {
        fprintf(events, "[%" PRIu64 "] ", unixtime(milli));
        fprintf(events, "LogS: %" PRIu64 " us\n", dt);
    }
    fflush(events);
}

void logstepbin()
{
    steplog.push((char*) &sc, sizeof(sc));
}

void logparams()
{
    fprintf(paramlog, "%" PRIu64" %3d "
            "%6.2lf %9.5lf "
            "%6.2lf %6.2lf %6.2lf "
            "%6.2lf %6.2lf %6.2lf "
            "%6.2lf %6.2lf %6.2lf "
            "%6.2lf %6.2lf %6.2lf "
            "%4.2lf %4.2lf\n",
            sc.t, (int) params.freq,
            params.z1h, params.z2h,
            hpid.Kp, hpid.Ki, hpid.Kd,
            ppid.Kp, ppid.Ki, ppid.Kd,
            rpid.Kp, rpid.Ki, rpid.Kd,
            zpid.Kp, zpid.Ki, zpid.Kd,
            params.gz_lpf, params.gz_wam);
    fflush(paramlog);
}

double trim(double val, double min, double max)
{
    return val > max ? max : val < min ? min : val;
}
