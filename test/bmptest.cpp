#include <BMP280.h>
#include <iostream>

using namespace std;

int main()
{
    BMP280 bmp;
    if (!bmp.begin())
    {
        cout << "No BMP280 found.\n";
        return 1;
    }

    double temp = bmp.readTemperature();
    cout << temp << " *C" << endl;
    double pressure = bmp.readPressure();
    cout << pressure << " Pa" << endl;
    double altitude = bmp.readAltitude();
    cout << altitude << " m" << endl;

    return 0;
}
