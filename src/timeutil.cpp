#include <sys/time.h>
#include <inttypes.h>
#include <timeutil.h>

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
    uint64_t start = unixtime(micro);
    uint64_t now = start;
    dt = dt * unit;
    while (start + dt > now)
    {
        now = unixtime(micro);
    }
}

void waituntil(uint64_t time, unit_t unit)
{
    uint64_t now = unixtime(micro);
    time = time * unit;
    while(now < time)
    {
        now = unixtime(micro);
    }
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
