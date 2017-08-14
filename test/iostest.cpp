#include <iostream>
#include <iomanip>
#include <chrono>
#include <thread>
#include <fstream>
#include <string>
#include <malloc.h>

int main()
{
    using namespace std;
    using namespace std::chrono;

    ofstream log, hex;
    log.open("log/log.txt");

    auto start = steady_clock::now();
    auto runtime = microseconds(0);
    auto dt = milliseconds(2);
    auto maxtime = seconds(30);

    size_t n = maxtime.count() * 1000 / dt.count();
    uint64_t* buffer = (uint64_t*) malloc(sizeof(uint64_t) * n);

    size_t count = 0;

    while (runtime < maxtime && count < n)
    {
        auto now = steady_clock::now();
        buffer[count] = duration_cast<milliseconds>(now - start).count();
        while (start + runtime < now)
            runtime += dt;

        while (now < start + runtime)
            now = steady_clock::now();
        count++;
    }

    for (size_t i = 0; i < count; i++)
    {
        log << buffer[i] << "\n";
    }
    log.close();
    return 0;
}
