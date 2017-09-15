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

// Set to true to print some debug messages, or false to disable them.
const bool debug = true;

pwm::pwm(uint8_t addr) : i2caddr(addr) { }

int pwm::begin()
{
    WIRE.begin();
    reset();
}


void Adafruit_PWMServoDriver::reset(void)
{
    i2c.write8(PCA9685_MODE1, 0x0);
}

void Adafruit_PWMServoDriver::setPWMFreq(float freq)
{
    std::cout << "Attempting to set freq " << freq << std::endl;
    freq *= 0.9;  // Correct for overshoot in the freq
    float prescaleval = 25000000;
    prescaleval /= 4096;
    prescaleval /= freq;
    prescaleval -= 1;
    if (debug)
        std::cout << "Estimated pre-scale: " << prescaleval << std::endl;
    uint8_t prescale = floor(prescaleval + 0.5);
    if (debug) std::cout << "Final pre-scale: " << prescale << std::endl;
  
    uint8_t oldmode = i2c.read8(PCA9685_MODE1);
    uint8_t newmode = (oldmode&0x7F) | 0x10; // sleep
    i2c.write8(PCA9685_MODE1, newmode); // go to sleep
    i2c.write8(PCA9685_PRESCALE, prescale); // set the prescaler
    i2c.write8(PCA9685_MODE1, oldmode);
    
    std::this_thread::sleep_for(std::chrono::milliseconds(5));

    // This sets the MODE1 register to turn on auto increment.
    i2c.write8(PCA9685_MODE1, oldmode | 0xa1);
    std::cout << "Mode now 0x" << std::hex
              << i2c.read8(PCA9685_MODE1) << std::endl;
}

void Adafruit_PWMServoDriver::setPWM(uint8_t num, uint16_t on, uint16_t off)
{  
    WIRE.beginTransmission(_i2caddr);
    WIRE.write(LED0_ON_L+4*num);
    WIRE.write(on);
    WIRE.write(on>>8);
    WIRE.write(off);
    WIRE.write(off>>8);
    WIRE.endTransmission();
}

// Sets pin without having to deal with on/off tick placement and properly handles
// a zero value as completely off.  Optional invert parameter supports inverting
// the pulse for sinking to ground.  Val should be a value from 0 to 4095 inclusive.
void Adafruit_PWMServoDriver::setPin(uint8_t num, uint16_t val, bool invert)
{
    // Clamp value between 0 and 4095 inclusive.
    val = std::min(val, 4095);
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
