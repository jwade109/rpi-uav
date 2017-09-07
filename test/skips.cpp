#include <iostream>
#include <iomanip>
#include <fstream>
#include <string>
#include <assert.h>

#include <uavcore.h>

int main(int argc, char** argv)
{
    std::string infile = "log/data.bin";
    if (argc > 1) infile = argv[1];

    std::ifstream bin;
    bin.open(infile, std::ios::in | std::ios::binary);
    if (!bin)
    {
        std::cerr << "Invalid filename." << std::endl;
        return 2;
    }

    std::streampos fsize = bin.tellg();
    bin.seekg(0, std::ios::end);
    fsize = bin.tellg() - fsize;
    bin.seekg(0, std::ios::beg);

    char* bytes = new char[fsize];
    bin.read(bytes, fsize);

    uav::freq_t f = *((uav::freq_t*) bytes);
    unsigned int dt = 1000/f;
    std::cerr << "Detected frequency of " << f
              << " Hz (dt = " << dt << " ms)" << std::endl;
    
    unsigned int maxdt = 0, mindt = -1;
    unsigned long long ts = 0, prev, skips = 0;

    std::cout.setf(std::ios::fixed, std::ios::floatfield);
    std::cout.precision(3);

    int n = ((int) fsize - uav::param_size)/uav::state_size;
    if (n == 0)
    {
        std::cout << "File does not contain any iterations." << std::endl;
        return 0;
    }
    for (int i = 0; i < n; i++)
    {
        if (i > 0) prev = ts;
        ts = *((uint64_t*) (bytes + uav::param_size + i * uav::state_size));

        if (i > 0 && ts - prev > maxdt) maxdt = ts - prev;
        if (i > 0 && ts - prev < mindt) mindt = ts - prev;

        if (i > 0 && ((ts - prev) != dt))
        {
            std::cout << std::setw(7) << prev/1000.0
                      << " -> " << std::setw(5) << ts/1000.0
                      << ": dt = " << ts - prev << std::endl;
            skips++;
        }
    }

    std::cout << "Finished. " << skips << " timing errors over "
              << n << " iterations, or " << ts/1000.0
              << " seconds." << std::endl
              << "Max dt: " << maxdt
              << ", min dt: " << mindt << std::endl;

    delete[] bytes;
}
