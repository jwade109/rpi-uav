#include <chrono>
#include <fstream>
#include <iostream>
#include <thread>

int main()
{
    using namespace std::chrono;

    std::cout << "Current epoch time: ";
    auto epoch = system_clock::now().time_since_epoch();
    std::cout << duration_cast<seconds>(epoch).count() << " s" << std::endl;
    std::cout << "Time since boot: ";
    auto other = steady_clock::now().time_since_epoch();
    std::cout << duration_cast<seconds>(other).count() << " s" << std::endl;

    std::cout << "Wait for 2500 ms:\t" << std::flush;
    steady_clock::time_point t1 = steady_clock::now();
    std::this_thread::sleep_until(t1 + milliseconds(2500));
    steady_clock::time_point t2 = steady_clock::now();
    nanoseconds ns1 = duration_cast<nanoseconds>(t2 - t1);
    std::cout << ns1.count() << " ns" << std::endl;

    std::cout << "Wait for 34123 us:\t" << std::flush;
    auto t3 = steady_clock::now();
    std::this_thread::sleep_until(t3 + microseconds(34123));
    auto t4 = steady_clock::now();
    auto ns2 = duration_cast<nanoseconds>(t4 - t3);
    std::cout << ns2.count() << " ns" << std::endl;

    std::cout << "Wait for 10 ms, 20 times:\t" << std::flush;
    nanoseconds ns3;
    for (int i = 0; i < 20; i++)
    {
        auto t5 = steady_clock::now();
        std::this_thread::sleep_until(t3 + milliseconds(10));
        auto t6 = steady_clock::now();
        ns3 = duration_cast<nanoseconds>(t6 - t5);
    }
    std::cout << ns3.count() << " ns" << std::endl;

    std::ofstream log;
    log.open("log/test.txt", std::ios::out);

    std::cout << "Print 300 lines at start + 5*i ms" << std::endl;
    auto start = steady_clock::now();
    auto wait = milliseconds(5);
    auto time = milliseconds(0);
    for (int i = 0; i < 300; i++)
    {
        auto real = duration_cast<milliseconds>(steady_clock::now() - start);
        log << real.count() << std::endl;
        time += wait;
        std::this_thread::sleep_until(start + time);
    }
    std::cout << "Finished." << std::endl;

    return 0;
}
