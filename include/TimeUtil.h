#ifndef TIMEUTIL_H
#define TIMEUTIL_H

#include <sys/time.h>
#include <inttypes.h>

typedef enum
{
    SEC = 1000000, MILLI = 1000, MICRO = 1,
}
unit_t;

uint64_t getUnixTime(unit_t precision);

void waitFor(uint64_t dt, unit_t unit);

void waitUntil(uint64_t time, unit_t unit);

#endif
