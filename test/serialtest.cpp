#include <SerialIMU.h>
#include <stdio.h>
#include <TimeUtil.h>

int main(int argc, char** argv)
{
    const int rate = 100; // hz

    SerialIMU imu;
    int start = imu.begin();
    if (start != 0)
    {
        printf("Error initializing IMU: %d\n", start);
        return 1;
    }
    waitFor(1, SEC);
    for (;;)
    {
        Message m = imu.get();
        printf("%lf\t%lf\t%lf\n", m.heading, m.pitch, m.roll);
        waitFor(1000/rate, MILLI);
    }
    return 0;
}
