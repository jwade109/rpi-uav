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
    bool x = bno.begin();
    bool y = bmp.begin();

    if (!x || !y)
    {
        Serial.println("WARN");
        for(;;)
        {
            if (!x)
            Serial.print("BNO ERROR ");
            if (!y)
            Serial.print("BMP ERROR ");
            Serial.println();
            delay(1000);
        }
    }

    bno.setExtCrystalUse(true);

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
    Serial.print(-euler.y() * 180/PI);
    Serial.print(' ');
    Serial.print(euler.z() * 180/PI);
    Serial.print(' ');

    uint8_t s, g, a, m;
    bno.getCalibration(&s, &g, &a, &m);
    Serial.print(s + (g << 2) + (a << 4) + (m << 6));
    Serial.print(' ');

    Serial.print(bmp.readPressure());
    Serial.print(' ');

    imu::Vector<3> accel = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
    Serial.print(-accel.y());
    Serial.print(' ');
    Serial.print(accel.x());
    Serial.print(' ');
    Serial.print(accel.z());
    Serial.print(' ');

    Serial.println('>');
}
