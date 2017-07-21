#include <BMP085.h>
#include <stdio.h>

using namespace std;

int main()
{
    BMP085 bmp;

    int status = bmp.begin(0x77);

    if (status)
    {
        printf("BMP085:\n");
        printf("%lf *C\n", bmp.temperature());
        printf("%lf Pa\n", bmp.pressure());
        printf("%lf m\n", bmp.altitude());
    }
    else printf("No BMP085 found.\n");

    return 0;
}
