#include <BMP085.h>
#include <TimeUtil.h>
#include <stdio.h>

int main()
{
    BMP085 bmp;

    int status = bmp.begin(0x77);

    for (; status == 0;)
    {
        printf("[%" PRIu64 "] ", getUnixTime(MILLI));
        printf("%.02lf *C\t", bmp.getTemperature());
        printf("%.02lf Pa\t", bmp.getPressure());
        printf("%.02lf m\n", bmp.getAltitude());
        waitFor(10, MILLI);
    }
    if (status) printf("No BMP085 found. (%d)\n", status);

    return 0;
}
