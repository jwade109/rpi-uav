#include <iostream>
#include <iomanip>
#include <fstream>
#include <string>
#include <assert.h>
#include <bitset>
#include <algorithm>

#include <uavcore.h>

int main(int argc, char** argv)
{
    if (argc < 4)
    {
        std::cerr << "usage: [input file] "
                     "[output file] [format]" << std::endl;
        return 1;
    }

    std::string m(argv[3]);
    std::reverse(m.begin(), m.end());
    std::bitset<uav::statelen> b(m);
    uint64_t mask = b.to_ullong();
    std::ifstream bin;
    std::ofstream txt;
    bin.open(argv[1], std::ios::in | std::ios::binary);
    if (!bin)
    {
        std::cerr << "Invalid filename." << std::endl;
        return 2;
    }
    txt.open(argv[2], std::ios::out | std::ios::binary);
    if (!txt)
    {
        std::cerr << "Could not create text file." << std::endl;
        return 3;
    }

    std::streampos fsize = bin.tellg();
    bin.seekg(0, std::ios::end);
    fsize = bin.tellg() - fsize;
    bin.seekg(0, std::ios::beg);

    char* bytes = new char[fsize];
    bin.read(bytes, fsize);
    uav::Param p;
    uav::from_buffer(p, bytes);
    txt << uav::pheader() << std::endl;
    txt << uav::to_string(p) << std::endl << std::endl;
    txt << uav::sheader(mask) << std::endl; 
    int n = uav::paramlen;
    while (n < fsize)
    {
        uav::State s;
        int ret = from_buffer(s, bytes + n);
        assert(ret == uav::statelen);
        txt << uav::to_string(s, mask) << std::endl;
        n += uav::statelen;
    }
    delete[] bytes;
}

