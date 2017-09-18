#ifndef MOTOR_H
#define MOTOR_H

#include <iostream>
#include <cstdlib>
#include <thread>

#include <pwm.h>

class motor
{
    public:

    motor(pwm_driver& pwm, uint8_t pin);
    ~motor();

    uint16_t get();
    void set(uint16_t level);
    void kill();
    
    private:

    void work();

    pwm_driver& pwm;
    bool setflag, cont;
    uint8_t pin;
    uint16_t level;
    std::thread setter;
};

#endif
