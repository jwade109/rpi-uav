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

// modified by Wade Foster for Raspberry Pi on 9/17/2017

#ifndef PWM_H
#define PWM_H

#include <inttypes.h>
#include <i2c.h>

const uint8_t PCA9685_SUBADR1   = 0x2;
const uint8_t PCA9685_SUBADR2   = 0x3;
const uint8_t PCA9685_SUBADR3   = 0x4;

const uint8_t PCA9685_MODE1     = 0x0;
const uint8_t PCA9685_PRESCALE  = 0xFE;

const uint8_t LED0_ON_L         = 0x6;
const uint8_t LED0_ON_H         = 0x7;
const uint8_t LED0_OFF_L        = 0x8;
const uint8_t LED0_OFF_H        = 0x9;

const uint8_t ALLLED_ON_L       = 0xFA;
const uint8_t ALLLED_ON_H       = 0xFB;
const uint8_t ALLLED_OFF_L      = 0xFC;
const uint8_t ALLLED_OFF_H      = 0xFD;

class pwm_driver
{
    public:

    int begin(uint8_t addr);
    void reset(void);
    void setPWMFreq(float freq);
    void setPWM(uint8_t num, uint16_t on, uint16_t off);
    void setPin(uint8_t num, uint16_t val, bool invert=false);

    private:
    
    i2cdev i2c;
};

#endif
