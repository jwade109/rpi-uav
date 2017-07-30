#ifndef TIMEUTIL_H
#define TIMEUTIL_H

#include <inttypes.h>

typedef enum
{
    sec = 1000000, milli = 1000, micro = 1,
}
unit_t;

uint64_t unixtime(unit_t precision = milli);

void waitfor(uint64_t dt, unit_t unit = milli);

void waituntil(uint64_t time, unit_t unit = milli);

uint64_t timer(unit_t precision = milli);

#endif
