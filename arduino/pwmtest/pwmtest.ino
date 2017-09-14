/***************************************************
  This is an example for our Adafruit 16-channel PWM & Servo driver
  PWM test - this will drive 16 PWMs in a 'wave'

  Pick one up today in the adafruit shop!
  ------> http://www.adafruit.com/products/815

  These displays use I2C to communicate, 2 pins are required to
  interface. For Arduino UNOs, thats SCL -> Analog 5, SDA -> Analog 4

  Adafruit invests time and resources providing this open source code,
  please support Adafruit and open-source hardware by purchasing
  products from Adafruit!

  Written by Limor Fried/Ladyada for Adafruit Industries.
  BSD license, all text above must be included in any redistribution
 ****************************************************/

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm;

void setup()
{
    Serial.begin(115200);
    pwm.begin();
    pwm.setPWMFreq(1600); // This is the maximum PWM frequency

    for (int i = 0; i < 16; i++)
        pwm.setPWM(i, 0, 0);
}

void loop()
{
    for (int i = 0; i < 4; i++)
    {
        pwm.setPWM(i * 4, 0, random(0,4096));
        delay(random(20,100));
    }
}
