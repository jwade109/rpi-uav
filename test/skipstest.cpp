#include <iostream>
#include <iomanip>
#include <fstream>
#include <string>

int main(int argc, char** argv)
{
    if (argc < 2)
    {
        std::cerr << "Please provide a filename." << std::endl;
        return 1;
    }

    std::ifstream log;
    log.open(argv[1], std::ios::in);
    if (!log)
    {
        std::cerr << "Invalid filename." << std::endl;
        return 2;
    }

    unsigned long long ts, first, last, i = 0;
    while (log)
    {
        log >> ts;
        if (log.eof())
        {
            last = ts;
            break;
        }
        if (i == 0) first = ts;
        while (log.get() != '\n' && !log.eof());
        if (log.eof())
            last = ts;
        i++;
    }
    std::cout << first << " " << last << " " << i << std::endl;
    unsigned long long avgdt = (last - first)/(i - 2);

    std::cerr << "Detected frequency of " << 1000.0/avgdt
              << " Hz (dt = " << avgdt
              << "ms). Correct? [y/n] " << std::flush;
    std::string r;
    std::getline(std::cin, r);
    unsigned int dt;
    if (!(r == "" || r == "y"))
    {    
        std::cerr << "Correct dt (ms): " << std::flush;
        std::cin >> dt;
    }
    else
    {
        dt = avgdt;
    }

    std::cout << "dt = " << dt << "ms, f = "
              << 1000/dt << " Hz" << std::endl;

    unsigned int maxdt = 0, mindt = -1;
    unsigned long long prev, skips = i = 0;

    log.clear();
    log.seekg(0, std::ios::beg);

    std::cout.setf(std::ios::fixed, std::ios::floatfield);
    std::cout.precision(3);

    while (log)
    {
        if (i > 0) prev = ts;
        log >> ts;
        if (log.eof())
        {
            break;
        }
        
        if (i > 0 && ts - prev > maxdt) maxdt = ts - prev;
        if (i > 0 && ts - prev < mindt) mindt = ts - prev;

        if (i > 0 && ((ts - prev) != dt))
        {
            std::cout << std::setw(7) << prev/1000.0
                      << " -> " << std::setw(5) << ts/1000.0
                      << ": dt = " << ts - prev << std::endl;
            skips++;
        }

        // skip until the next line
        while (log.get() != '\n' && !log.eof());
        i++;
    }
    std::cout << "Finished. " << skips << " timing errors over "
              << i << " iterations, or " << (last - first + dt)/1000.0
              << " seconds." << std::endl
              << "Max dt: " << maxdt
              << ", min dt: " << mindt << std::endl;
}

