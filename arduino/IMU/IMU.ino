#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <Adafruit_BMP280.h>
#include <Adafruit_BMP085_U.h>
#include <utility/imumaths.h>

Adafruit_BNO055 bno;
Adafruit_BMP280 bmp;

void setup(void)
{
    Serial.begin(115200);
    int x = bno.begin();
    int y = bmp.begin();

    if (x + y < 2)
    {
        for(;;)
        {
            Serial.print("! BNO055: ");
            Serial.print(x);
            Serial.print(" BMP280: ");
            Serial.print(y);
            Serial.println(" !");
            delay(1000);
        }
    }

    bno.setExtCrystalUse(true);

    Serial.print('#');
    delay(1000);
    Serial.print('#');
}

void loop(void)
{
    // <TIME HDG PITCH ROLL CALIB TEMP PRESS>

    Serial.print('<');

    Serial.print(millis());
    Serial.print(' ');

    imu::Vector<3> euler = bno.getQuat().toEuler();
    double hdg = degrees(euler.x()) - 90;
    Serial.print(hdg < 0 ? hdg + 360 : hdg);
    Serial.print(' ');
    Serial.print(euler.y() * 180/PI);
    Serial.print(' ');
    Serial.print(euler.z() * 180/PI);
    Serial.print(' ');

    uint8_t s, g, a, m;
    bno.getCalibration(&s, &g, &a, &m);
    Serial.print(s + (g << 2) + (a << 4) + (m << 6));
    Serial.print(' ');

    Serial.print(bmp.readTemperature());
    Serial.print(' ');
    Serial.print(bmp.readPressure());

    Serial.println('>');
}
