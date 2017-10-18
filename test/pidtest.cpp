#include <iostream>
#include <iomanip>
#include <fstream>
#include <thread>
#include <cstdlib>
#include <cstring>
#include <cstdbool>
#include <chrono>

#include <pid.h>

bool verbose = false;
bool toFile = false;
int whack = -1;
double error_ext = 0;
std::ofstream dataFile;

int main(int argc, char** argv)
{
    using namespace std::chrono;
    typedef std::chrono::duration<double> fsec;

    double position = 0, setpoint = 10, P = 1, I = 0.02, D = 2;
    auto dt = milliseconds(10);

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
            sscanf(argv[i], "%lf", &error_ext);
        }
        else if (strcmp(argv[i], "--to-file") == 0 || strcmp(argv[i], "-f") == 0)
        {
            toFile = true;
        }
        else
        {
            std::cout << "invalid arguments. usage:"
                      << "-P [Kp] -I [Ki] -D [Kd]     tune individual controller gains\n"
                      << "-t --tune [Kp Ki Kd]        tune controller gain values\n"
                      << "-v --verbose                display P-I-D responses\n"
                      << "-i --initial [val]          set process variable initial value\n"
                      << "-s --setpoint [val]         set setpoint value\n"
                      << "-w --whack [val]            cause purturbation at given time\n"
                      << "-e --external [val]         simulate discrepancy between plant and model\n"
                      << "-f --to-file                export data to a text file\n" << std::endl;
            return 1;
        }
    }

    if (toFile)
    {
        dataFile.open("pid.txt", std::ios::out);
    }

    pid_controller control(P, I, D, -1);
    double vel = 0;
    int count = 0;
    
    if (verbose)
    {
        std::cout << "Time\tP\tI\tD\tOP\tPV\t";
    }
    else
    {
        std::cout << "Time\tOP\tPV\t";
    }
    std::cout << "Kd: " << std::setprecision(3) << control.Kp << " "
              << "Ki: " << std::setprecision(3) << control.Ki << " "
              << "Kd: " << std::setprecision(3) << control.Kd << " "
              << "Setpoint: " << std::setprecision(3) << setpoint << std::endl;
    std::this_thread::sleep_for(seconds(2));
    
    auto start = steady_clock::now();

    for (auto t = milliseconds(0); t < minutes(1); t += dt)
    {
        double resp = control.seek(position, setpoint,
                duration_cast<fsec>(dt).count());
        vel += (resp - error_ext) * duration_cast<fsec>(dt).count();
        if (count == whack * 100)
        {
            printf("-- WHACK --\n");
            vel = -setpoint * 2;
        }
        position += vel * duration_cast<fsec>(dt).count();
        printf("%.2f:\t", duration_cast<fsec>(t).count());
        if (verbose)
        {
            printf("%.2f\t%.2f\t%.2f\t%.2f\t",
                control.p_response, control.i_response,
                control.d_response, resp);
        }
        else
        {
            printf("%.2f\t", resp);
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
        if (toFile)
        {
            dataFile << duration_cast<fsec>(t).count()
                     << "\t" << position << std::endl;
            dataFile.flush();
        }
        std::this_thread::sleep_until(start + t + dt);
        count++;
    }
}
