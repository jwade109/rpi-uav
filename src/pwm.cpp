/*************************************************** 
  This is a library for our Adafruit 16-channel PWM & Servo driver

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

// modified by Wade Foster for Raspberry Pi on 9/14/2017

#include <pwm.h>
#include <iostream>
#include <thread>
#include <chrono>
#include <cmath>

pwm_driver::pwm_driver() : i2caddr(0) { }

int pwm_driver::begin(uint8_t addr)
{
    i2c = I2C(addr);
    int ret = i2c.ready();
    if (!ret)
    {
        std::cerr << "pwm: i2c init error: " << ret << std::endl;
        return 1;
    }
    return 0;
}

void pwm_driver::reset(void)
{
    i2c.write8(PCA9685_MODE1, 0x0);
}

void pwm_driver::setPWMFreq(float freq)
{
    freq *= 0.9;  // Correct for overshoot in the freq
    float prescaleval = 25000000;
    prescaleval /= 4096;
    prescaleval /= freq;
    prescaleval -= 1;
    uint8_t prescale = std::floor(prescaleval + 0.5);
  
    uint8_t oldmode = i2c.read8(PCA9685_MODE1);
    uint8_t newmode = (oldmode&0x7F) | 0x10; // sleep
    i2c.write8(PCA9685_MODE1, newmode); // go to sleep
    i2c.write8(PCA9685_PRESCALE, prescale); // set the prescaler
    i2c.write8(PCA9685_MODE1, oldmode);
    
    std::this_thread::sleep_for(std::chrono::milliseconds(5));

    // This sets the MODE1 register to turn on auto increment.
    i2c.write8(PCA9685_MODE1, oldmode | 0xa1);
}

void pwm_driver::setPWM(uint8_t num, uint16_t on, uint16_t off)
{
    uint8_t base = LED0_ON_L + 4*num;
    i2c.write8(base, on);
    i2c.write8(base + 1, on >> 8);
    i2c.write8(base + 2, off);
    i2c.write8(base + 3, off >> 8);
}

void pwm_driver::setPin(uint8_t num, uint16_t val, bool invert)
{
    // Clamp value between 0 and 4095 inclusive.
    val = std::min(val, (uint16_t) 4095);
    if (invert)
    {
        if (val == 0) setPWM(num, 4096, 0);
        else if (val == 4095)  setPWM(num, 0, 4096);
        else setPWM(num, 0, 4095-val);
    }
    else
    {
        if (val == 4095) setPWM(num, 4096, 0);
        else setPWM(num, 0, val);
    }
}
