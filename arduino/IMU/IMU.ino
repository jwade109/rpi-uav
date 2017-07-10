#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <Adafruit_BMP280.h>
#include <Adafruit_BMP085_U.h>
#include <utility/imumaths.h>

Adafruit_BNO055 bno = Adafruit_BNO055();
Adafruit_BMP280 bmp = Adafruit_BMP280();

long int last_millis = 0;

void setup(void)
{
    Serial.begin(115200);
    int x = bno.begin();
    int y = bmp.begin();

    if (x + y < 2)
    {
        for(;;)
        {
            Serial.print("! ");
            Serial.print(x * 10 + y);
            Serial.println(" !");
            delay(500);
        }
    }

    bno.setExtCrystalUse(true);

    Serial.print('#');
    delay(1000);
    Serial.print('#');
}

void loop(void)
{
    // <TIME HDG PITCH ROLL CALIB ALT>

    Serial.print('<');

    Serial.print(millis());
    Serial.print(' ');

    imu::Vector<3> euler = bno.getQuat().toEuler();
    Serial.print(euler.x() * 180/PI);
    Serial.print(' ');
    Serial.print(euler.y() * 180/PI);
    Serial.print(' ');
    Serial.print(euler.z() * 180/PI);
    Serial.print(' ');

    uint8_t s, g, a, m;
    bno.getCalibration(&s, &g, &a, &m);
    Serial.print((s << 6) + (g << 4) + (a << 2) + m);
    Serial.print(' ');

    Serial.print(bmp.readAltitude());

    Serial.println('>');
}
