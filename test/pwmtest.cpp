#include <iostream>
#include <pwm.h>
#include <motor.h>
#include <thread>
#include <chrono>

int main()
{
    pwm_driver pwm;
    int ret = pwm.begin(0x40);
    std::cout << "Connection: " << ret << std::endl;
    pwm.reset();
    pwm.setPWMFreq(1600);

    motor* m[4];
    for (int i = 0; i < 4; i++)
        m[i] = new motor(pwm, i * 4);

    int j = 0;
    while (j < 10)
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
