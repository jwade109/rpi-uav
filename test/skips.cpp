#include <iomanip>
#include <fstream>
#include <string>
#include <assert.h>

#include <monitor.h>

int main(int argc, char** argv)
{
    if (argc < 2)
    {
        std::cerr << "Please provide a filename." << std::endl;
        return 1;
    }

    std::ifstream bin;
    bin.open(argv[1], std::ios::in | std::ios::binary);
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

    uav::Freq f = *((uav::Freq*) bytes);
    unsigned int dt = 1000/f;
    std::cerr << "Detected frequency of " << f
              << " Hz (dt = " << dt << " ms)" << std::endl;
    
    unsigned int maxdt = 0, mindt = -1;
    unsigned long long ts, prev, skips = 0;

    std::cout.setf(std::ios::fixed, std::ios::floatfield);
    std::cout.precision(3);

    int n = ((int) fsize - uav::paramlen)/uav::statelen;
    for (int i = 0; i < n; i++)
    {
        if (i > 0) prev = ts;
        ts = *((uint64_t*) (bytes + uav::paramlen + i * uav::statelen));

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

/*

#include <iostream>
#include <iomanip>
#include <fstream>
#include <string>

int main(int argc, char** argv)
{

}
*/
