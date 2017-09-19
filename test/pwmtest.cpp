#include <iostream>
#include <pwm.h>
#include <motor.h>
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
    pwm.begin(0x40);
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

    uav::motor* m[4];
    for (int i = 0; i < 4; i++)
        m[i] = new uav::motor(pwm, i * 4);

    int j = 0;
    while (j < 3 && cont)
    {
        for (int i = 0; i < 4; i++)
        {
            m[i]->set(3000);
            std::this_thread::sleep_for(std::chrono::milliseconds(150));
            m[i]->kill();
        }
        j++;
    }

    return 0;
}
