#include <motor.h>
#include <chrono>

uav::motor::motor(pwm_driver& pwm, uint8_t pin) :
    pwm(pwm), setflag(true), cont(true),
    pin(pin), level(0),
    setter(&motor::work, this) { }

uav::motor::~motor()
{
    kill();
    std::this_thread::sleep_for(std::chrono::milliseconds(3));
    cont = false;
    if (setter.joinable()) setter.join();
}

uint16_t uav::motor::get() const { return level; }

void uav::motor::set(uint16_t level)
{
    this->level = level;
    setflag = true;
}

void uav::motor::kill()
{
    level = 0;
    setflag = true;
}

void uav::motor::work()
{
    while (cont)
        if (setflag)
        {
            pwm.setPin(pin, level, false);
            setflag = false;
        }
}
