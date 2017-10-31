#include <fstream>
#include <iostream>
#include <inttypes.h>
#include <chrono>

#include <motor.h>

template <typename T> Motor<T>::Motor(T initial) : cont(true),
    worker(&Motor<T>::work, this)
{
    set(initial);
}

template <typename T> Motor<T>::~Motor()
{
    cont = false;
    if (worker.joinable()) worker.join();

    std::ofstream out("out.txt");
    while (!msg.empty())
    {
        out << msg.front() << "\n";
        msg.pop_front();
    }
    out.close();
}

template <typename T> const T Motor<T>::get() const
{
    return value;
}

template <typename T> void Motor<T>::set(T x)
{
    value = x;
}

template <typename T> void Motor<T>::work()
{
    std::chrono::steady_clock::time_point last;
    while (cont)
    {
        auto now = std::chrono::steady_clock::now();
        auto wait = std::chrono::milliseconds(200);
        if (now > last + wait)
        {
            msg.push_back(std::to_string(value) + " " +
                std::to_string(now.time_since_epoch().count()));
            std::cout << value << std::endl;
            last = now;
        }
    }
}

template class Motor<char>;
template class Motor<int>;
template class Motor<uint8_t>;
template class Motor<uint16_t>;
template class Motor<uint32_t>;
template class Motor<uint64_t>;
template class Motor<float>;
template class Motor<double>;
