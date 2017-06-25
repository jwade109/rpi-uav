#include <sys/time.h>
#include <inttypes.h>
#include <TimeUtil.h>

uint64_t getUnixTime(unit_t precision)
{
    struct timeval now;
    gettimeofday(&now, 0);
    uint64_t time = (uint64_t) (now.tv_sec) * SEC +
        (uint64_t) now.tv_usec;
    time /= precision;
    return time;
}

void waitFor(uint64_t dt, unit_t unit)
{
    uint64_t start = getUnixTime(MICRO);
    uint64_t now = start;
    dt = dt * unit;
    while (start + dt > now)
    {
        now = getUnixTime(MICRO);
    }
}

void waitUntil(uint64_t time, unit_t unit)
{
    uint64_t now = getUnixTime(MICRO);
    time = time * unit;
    while(now < time)
    {
        now = getUnixTime(MICRO);
    }
}
