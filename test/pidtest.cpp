#include <stdio.h>
#include <iostream>
#include <fstream>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>

#include <pid.h>
#include <timeutil.h>

using namespace std;

bool verbose = false;
bool toFile = false;
int whack = -1;
double external_error = 0;
ofstream dataFile;

int main(int argc, char** argv)
{
    double position = 0, setpoint = 10, dt = 0.01;
    double P = 1, I = 0.02, D = 2;

    for (int i = 1; i < argc; i++)
    {
        if (strcmp(argv[i], "--tune") == 0 || strcmp(argv[i], "-t") == 0)
        {
            i++;
            sscanf(argv[i], "%lf", &P);
            i++;
            sscanf(argv[i], "%lf", &I);
            i++;
            sscanf(argv[i], "%lf", &D);
        }
        else if (strcmp(argv[i], "-P") == 0)
        {
            i++;
            sscanf(argv[i], "%lf", &P);
        }
        else if (strcmp(argv[i], "-I") == 0)
        {
            i++;
            sscanf(argv[i], "%lf", &I);
        }
        else if (strcmp(argv[i], "-D") == 0)
        {
            i++;
            sscanf(argv[i], "%lf", &D);
        }
        else if (strcmp(argv[i], "--initial") == 0 || strcmp(argv[i], "-i") == 0)
        {
            i++;
            sscanf(argv[i], "%lf", &position);
        }
        else if (strcmp(argv[i], "--setpoint") == 0 || strcmp(argv[i], "-s") == 0)
        {
            i++;
            sscanf(argv[i], "%lf", &setpoint);
        }
        else if (strcmp(argv[i], "--verbose") == 0 || strcmp(argv[i], "-v") == 0)
        {
            verbose = 1;
        }
        else if (strcmp(argv[i], "--whack") == 0 || strcmp(argv[i], "-w") == 0)
        {
            i++;
            sscanf(argv[i], "%d", &whack);
        }
        else if (strcmp(argv[i], "--external") == 0 || strcmp(argv[i], "-e") == 0)
        {
            i++;
            sscanf(argv[i], "%lf", &external_error);
        }
        else if (strcmp(argv[i], "--to-file") == 0 || strcmp(argv[i], "-f") == 0)
        {
            toFile = true;
        }
        else
        {
            printf("invalid arguments. usage:\n");
            printf("-P [Kp] -I [Ki] -D [Kd]     tune individual controller gains\n");
            printf("-t --tune [Kp Ki Kd]        tune controller gain values\n");
            printf("-v --verbose                display P-I-D responses\n");
            printf("-i --initial [val]          set process variable initial value\n");
            printf("-s --setpoint [val]         set setpoint value\n");
            printf("-w --whack [val]            cause purturbation at given time\n");
            printf("-e --external [val]         simulate discrepancy between plant and model\n");
            printf("-f --to-file                export data to a text file\n");
            return 1;
        }
    }

    if (toFile)
    {
        dataFile.open("pid.txt", ios::out);
    }

    PID control(P, I, D, -1);
    double velocity = 0;
    int count = 0;
    
    if (verbose)
    {
        printf("Time\tP\tI\tD\tOP\tPV\t");
    }
    else
    {
        printf("Time\tOP\tPV\t");
    }
    printf("Kd: %.2f\tKi: %.2f\tKd: %.2f\tSetpoint: %.2f\n",
        control.Kp, control.Ki, control.Kd, setpoint);
    waitfor(2, sec);
    
    uint64_t start_time = unixtime(micro);

    for (int iter = 0; iter < 60/dt; iter++)
    {
        double t = iter * dt;
        double response = control.seek(position, setpoint, dt);
        velocity += (response - external_error) * dt;
        if (count == whack * 100)
        {
            printf("-- WHACK --\n");
            velocity = -setpoint * 2;
        }
        position += velocity * dt;
        printf("%.2f:\t", t);
        if (verbose)
        {
            printf("%.2f\t%.2f\t%.2f\t%.2f\t",
                control.p_response, control.i_response,
                control.d_response, response);
        }
        else
        {
            printf("%.2f\t", response);
        }
        printf("%.2f\t", position);
        for (int i = 0; i < 100; i++)
        {
            if (i == 50)
                printf("*");
            else if (i < position * 50 / setpoint)
                printf("|");
            else if (i < 50)
                printf(" ");
        }
        printf("\n");
        uint64_t time = start_time + (t + dt) * sec;
        if (toFile)
        {
            dataFile << t << "   " << position << endl;
            dataFile.flush();
        }
        waituntil(time, micro);
        count++;
    }
}
