#include <stdio.h>
#include <SerialIMU.h>
#include <timeutil.h>

int main(int argc, char** argv)
{
    const int rate = 100; // hz

    SerialIMU imu;
    int start = imu.begin();
    if (start != 0)
    {
        fprintf(stderr, "Error initializing IMU: %d\n", start);
        return 1;
    }
    waitfor(1, sec);
    for (;;)
    {
        Message m = imu.get();
        printf("%.02lf\t%.02lf\t%.02lf\t%.02lf\n",
            m.heading, m.pitch, m.roll, m.alt);
        waitfor(1000/rate, milli);
    }
    return 0;
}
