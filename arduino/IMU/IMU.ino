#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <Adafruit_BMP280.h>
#include <Adafruit_BMP085_U.h>
#include <utility/imumaths.h>

Adafruit_BNO055         bno  = Adafruit_BNO055();
Adafruit_BMP085_Unified bmpA = Adafruit_BMP085_Unified();
Adafruit_BMP280         bmpB = Adafruit_BMP280();

void setup(void)
{
    Serial.begin(115200);
    int x = bno.begin();
    int y = bmpA.begin();
    int z = bmpB.begin(0x76);

    if (x + y + z < 3)
    {
        for(;;)
        {
            Serial.print("! ");
            Serial.print(x * 100 + y * 10 + z);
            Serial.println(" !");
            delay(500);
        }
    }
    Serial.print('#');

    bno.setExtCrystalUse(true);
}

void loop(void)
{
    // <TIME HDG PITCH ROLL CALIB ALTA ALTB>

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

    float pressure;
    bmpA.getPressure(&pressure);

    Serial.print(bmpA.pressureToAltitude(101325, pressure));
    Serial.print(' ');
    Serial.print(bmpB.readAltitude());

    Serial.println('>');
}
