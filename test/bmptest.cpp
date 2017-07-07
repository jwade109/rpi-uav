#include <BMP085.h>
#include <BMP280.h>
#include <iostream>

using namespace std;

int main()
{
    BMP085 bmp1;
    BMP280 bmp2;

    int status = bmp1.begin(0x77) + 2 * bmp2.begin(0x76);

    if (status == 1 || status == 3)
    {
        cout << "BMP085:" << endl;
        double temp = bmp1.temperature();
        cout << temp << " *C" << endl;
        double pressure = bmp1.pressure();
        cout << pressure << " Pa" << endl;
        double altitude = bmp1.altitude();
        cout << altitude << " m" << endl;
    }
    else cout << "No BMP085 found." << endl;

    if (status == 2 || status == 3)
    {
        cout << "BMP280:" << endl;
        double temp = bmp2.temperature();
        cout << temp << " *C" << endl;
        double pressure = bmp2.pressure();
        cout << pressure << " Pa" << endl;
        double altitude = bmp2.altitude();
        cout << altitude << " m" << endl;
    }
    else cout << "No BMP280 found." << endl;

    return 0;
}
