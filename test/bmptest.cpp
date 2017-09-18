#include <stdio.h>
#include <bmp.h>
#include <chrono>
#include <thread>

int main()
{
    using namespace std::chrono;

    uav::bmp085 bmp;
    int status = bmp.begin(0x77);
    if (status)
    {
        printf("BMP init error: %d\n", status);
        return 1;
    }

    auto start = steady_clock::now();
    auto wait = milliseconds(10);
    auto dur = milliseconds(0);
    while (dur < minutes(2))
    {
        printf("[%llu]\t", dur.count());
        printf("%.02lf *C\t", bmp.getTemperature());
        printf("%.02lf Pa\t", bmp.getPressure());
        printf("%.02lf m\n", bmp.getAltitude());
        dur += wait;
        std::this_thread::sleep_until(start + dur);
    }
    if (status) printf("No BMP085 found. (%d)\n", status);

    return 0;
}
