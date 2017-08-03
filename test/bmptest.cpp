#include <stdio.h>
#include <bmp.h>
#include <timeutil.h>

int main()
{
    BMP085 bmp;

    int status = bmp.begin(0x77);

    uint64_t start = unixtime(milli); 
    for (; status == 0;)
    {
        printf("[%" PRIu64 "] ", unixtime(milli));
        printf("%.02lf *C\t", bmp.getTemperature());
        printf("%.02lf Pa\t", bmp.getPressure());
        printf("%.02lf m\n", bmp.getAltitude());
        waituntil(start+=10, milli);
    }
    if (status) printf("No BMP085 found. (%d)\n", status);

    return 0;
}
