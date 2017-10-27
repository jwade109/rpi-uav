#include <iostream>
#include <iomanip>
#include <fstream>
#include <string>
#include <assert.h>
#include <bitset>
#include <algorithm>

#include <uav/filter>
#include <uav/logging>

int main(int argc, char** argv)
{
    using namespace uav;

    std::string outfile("log/vel.txt"), infile("log/data.bin");
    if (argc > 2) outfile = argv[2];
    if (argc > 1) infile = argv[1];

    std::ifstream bin;
    std::ofstream txt;
    bin.open(infile, std::ios::in | std::ios::binary);
    if (!bin)
    {
        std::cerr << "Invalid filename." << std::endl;
        return 2;
    }
    txt.open(outfile, std::ios::out | std::ios::binary);
    if (!txt)
    {
        std::cerr << "Could not create text file." << std::endl;
        return 3;
    }

    std::streampos fsize = bin.tellg();
    bin.seekg(0, std::ios::end);
    fsize = bin.tellg() - fsize;
    bin.seekg(0, std::ios::beg);

    txt << std::fixed << std::setprecision(4)
        << std::setw(12) << "time" << std::setw(12) << "x"
        << std::setw(12) << "y"    << std::setw(12) << "z"
        << std::setw(12) << "vx"   << std::setw(12) << "vy"
        << std::setw(12) << "vz"   << std::setw(12) << "ax"
        << std::setw(12) << "ay"   << std::setw(12) << "az"
        << std::endl;

    char* bytes = new char[fsize];
    bin.read(bytes, fsize);
    int n = param::size;
    derivative<1> ddt, vx, vy, vz;
    derivative<2> ax, ay, az;

    while (n < fsize)
    {
        state s = deserialize(wrap<state::size>(bytes + n));
        auto dt = ddt(s.time[0]/1000.0);
        txt << std::setw(12) << s.time[0]/1000.0
            << std::setw(12) << s.position[0]
            << std::setw(12) << s.position[1]
            << std::setw(12) << s.position[2]
            << std::setw(12) << vx(s.position[0], dt)
            << std::setw(12) << vy(s.position[1], dt)
            << std::setw(12) << vz(s.position[2], dt)
            << std::setw(12) << ax(s.position[0], dt)
            << std::setw(12) << ay(s.position[1], dt)
            << std::setw(12) << az(s.position[2], dt)
            << std::endl;
        n += state::size;
    }
    delete[] bytes;
    txt << std::flush;
}

