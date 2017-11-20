#include <iostream>
#include <pwm.h>
#include <thread>
#include <chrono>
#include <math.h>
#include <signal.h>

bool cont = true;

void sigint(int signal)
{
    cont = false;
}

int main()
{
    signal(SIGINT, sigint);

    pwm_driver pwm;
    if (pwm.begin(0x40) > 0) return 1;
    pwm.reset();
    pwm.setPWMFreq(1600);
    {
        double t = 0;
        while (t < 20 && cont)
        {
            for (int i = 0; i < 16; i++)
            {
                pwm.setPin(i, 2000 * pow(sin(t + i * 0.2), 2), false);
            }
            t += 0.05;
        }
    }

    return 0;
}
