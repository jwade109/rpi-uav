#include <wiringPi.h>
#include <softPwm.h>
#include <stdio.h>
#include <iostream>
#include <stdint.h>
#include <stdbool.h>
#include <TimeUtil.h>

using namespace std;

int main()
{
    wiringPiSetupGpio();
    bool success = !softPwmCreate(18, 0, 100);
    if (!success) return 1;

    for(int i = 0; i <= 100; i++)
    {
        softPwmWrite(18, i);
        cout << i << endl;
        waitFor((i == 4 ? 1000 : 100), MILLI);
    }
    softPwmWrite(18, 0);
    waitFor(10, MILLI);
    return 0;
}
