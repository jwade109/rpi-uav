#include <filters.h>
#include <timeutil.h>
#include <stdio.h>

int main()
{
    float point = 150;
    RateLimiter rt(100, point); // limit rate to 300 Hz
    uint64_t wait = 100, start = unixtime();
    float dt = 1.0 * wait / milli;
    printf("%" PRIu64 ":\t%f\n", start, point);
    while (point > 0)
    {
        point = rt.step(0, dt);
        start+=wait;
        waituntil(start);
        printf("%" PRIu64 ":\t%f\n", start, point);
    }
    return 0;
}
