#include <sys/time.h>
#include <inttypes.h>
#include <timeutil.h>
#include <chrono>
#include <thread>

uint64_t unixtime(unit_t precision)
{
    struct timeval now;
    gettimeofday(&now, 0);
    uint64_t time = (uint64_t) (now.tv_sec) * sec +
        (uint64_t) now.tv_usec;
    time /= precision;
    return time;
}

void waitfor(uint64_t dt, unit_t unit)
{
    if (unit == milli)
        std::this_thread::sleep_for(std::chrono::milliseconds(dt));
    else if (unit == micro)
        std::this_thread::sleep_for(std::chrono::microseconds(dt));
    else if (unit == sec)
        std::this_thread::sleep_for(std::chrono::seconds(dt));
}

void waituntil(uint64_t time, unit_t unit)
{
    uint64_t now = unixtime(micro);
    waitfor(time - now, micro);
}

uint64_t timer(unit_t unit)
{
    static uint64_t last;
    uint64_t dt, now = unixtime(micro);
    if (last != 0) dt = now - last;
    else dt = 0;
    last = now;
    return dt/unit;
}
