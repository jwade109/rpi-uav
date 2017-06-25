#include <iostream>
#include <inttypes.h>
#include <TimeUtil.h>

int main()
{
    double runtime = 23;
    double dt = 0.007;
    int maxiter = runtime/dt;
    uint64_t start = getUnixTime(MICRO);
    std::cout << start << std::endl;
    for (int i = 0; i < maxiter; i++)
    {
        double t = i * dt;
        std::cout << i << "\t" << t << std::endl;
        uint64_t time = start + (t + dt) * SEC;
        waitUntil(time, MICRO);
    }
    uint64_t end = getUnixTime(MICRO);
    std::cout << end << std::endl;
    std::cout << end - start << std::endl;
    return 0;

    /*

    std::cout << "Wait for 400 ms, 300 ms, 1 s, 2000 us" << std::endl;

    uint64_t time1 = getUnixTime(MICRO);

    waitFor(400, MILLI);
    uint64_t time2 = getUnixTime(MICRO);
    std::cout << (time2 - time1) << " us passed" << std::endl;

    waitFor(300, MILLI);
    time1 = getUnixTime(MICRO);
    std::cout << (time1 - time2) << " us passed" << std::endl;

    waitFor(1, SEC);
    time2 = getUnixTime(MICRO);
    std::cout << (time2 - time1) << " us passed" << std::endl;

    waitFor(2000, MICRO);
    time1 = getUnixTime(MICRO);
    std::cout << (time1 - time2) << " us passed" << std::endl;

    time1 = getUnixTime(MICRO);
    waitUntil(time1 + 1.5 * SEC, MICRO);
    time2 = getUnixTime(MICRO);
    std::cout << (time2 - time1) << " us passed" << std::endl;

    std::cout << "-------------------------------" << std::endl;

    double delta = 3; // sec
    int iter = 300;
    uint64_t dt = delta/iter * SEC; // us
    std::cout << "dt: " << dt << "  iter: " << iter << std::endl;
    std::cout << "ideal total delta: " << (delta * SEC) << std::endl;

    time1 = getUnixTime(MICRO);
    for (int i = 0; i < iter; i++)
    {
        waitFor(dt, MICRO);
    }
    time2 = getUnixTime(MICRO);
    std::cout << (time2 - time1) << " us passed" << std::endl;

    time1 = getUnixTime(MICRO);
    for (int i = 0; i < iter; i++)
    {
        uint64_t wait = (i + 1) * dt;
        waitUntil(time1 + wait, MICRO);
    }
    time2 = getUnixTime(MICRO);
    std::cout << (time2 - time1) << " us passed" << std::endl;

    */
}
