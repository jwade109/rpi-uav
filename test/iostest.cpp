#include <iostream>
#include <iomanip>
#include <chrono>
#include <thread>
#include <fstream>
#include <string>

int main()
{
    using namespace std;
    using namespace std::chrono;

    ofstream log, hex;
    log.open("log/log.txt");

    auto start = steady_clock::now();
    auto wait = milliseconds(5);
    int count = 0;
    for (auto i = milliseconds(0); i < seconds(2); i += wait)
    {
        auto now = steady_clock::now();
        log << count << " " << duration_cast<milliseconds>(now - start).count()
            << " Well perhaps fstream holds the answer to"
               " reliable logging since a very nasty bug"
               " threatens the future of filebuffer! Let's"
               " try going faster and writing more to"
               " this newfangled c++ construct" << endl;
        std::this_thread::sleep_until(start + i + wait);
        count++;
    }
}
