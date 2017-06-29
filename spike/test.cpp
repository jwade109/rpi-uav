#include <BMP280.h>
#include <iostream>

using namespace std;

int main()
{
    BMP280 bmp;

    int init = bmp.begin();
    cout << "Init:\n" << init << endl;
    if (!init) return 1;

    double temp = bmp.readTemperature();
    cout << "Temperature = " << temp << endl;
    double pressure = bmp.readPressure();
    cout << "Pressure = " << pressure << endl;

    return 0;
}
