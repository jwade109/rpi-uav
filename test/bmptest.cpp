#include <BMP280.h>
#include <iostream>

using namespace std;

int main()
{
    BMP280 bmp;
    if (!bmp.begin(0x77))
    {
        cout << "No BMP280 found.\n";
        return 1;
    }

    double temp = bmp.readTemperature();
    cout << temp << " *C" << endl;
    double pressure = bmp.readPressure();
    cout << pressure << " Pa" << endl;
    double altitude = bmp.readAltitude(1017.83);
    cout << altitude << " m" << endl;

    return 0;
}
