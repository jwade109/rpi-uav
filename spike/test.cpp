#include <BMP280.h>
#include <iostream>

using namespace std;

int main()
{
    Adafruit_BMP280 bmp;

    cout << bmp.begin() << endl;
    cout << bmp.readTemperature() << endl;
    cout << bmp.readPressure() << endl;
    cout << bmp.readAltitude() << endl;

    return 0;
}
