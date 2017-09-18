#include <motor.h>
#include <chrono>

motor::motor(pwm_driver& pwm, uint8_t pin) :
    pwm(pwm), setflag(true), cont(true),
    pin(pin), level(0),
    setter(&motor::work, this) { }

motor::~motor()
{
    kill();
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
    cont = false;
    if (setter.joinable()) setter.join();
}

uint16_t motor::get() { return level; }

void motor::set(uint16_t level)
{
    this->level = level;
    setflag = true;
}

void motor::kill()
{
    level = 0;
    setflag = true;
}

void motor::work()
{
    while (cont)
        if (setflag)
        {
            pwm.setPWM(pin, 0, level);
            setflag = false;
        }
}
