#ifndef MOTOR_H
#define MOTOR_H

#include <thread>
#include <deque>
#include <string>

template <typename T> class Motor
{
    public:

    Motor(T initial = 0);
    ~Motor();
    const T get() const;
    void set(T x);

    private:

    T value;
    bool cont;
    std::thread worker;
    std::deque<std::string> msg;

    void work();
};

#endif // MOTOR_H
