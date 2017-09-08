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
    using namespace uav;

    uint64_t mask = -1;
    std::string outfile("log/out.txt"), infile("log/data.bin");
    if (argc > 3)
    {
        std::string m(argv[3]);
        std::reverse(m.begin(), m.end());
        std::bitset<state::size> b(m);
        mask = b.to_ullong();
    }
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

    char* bytes = new char[fsize];
    bin.read(bytes, fsize);
    param p = from_binary(wrap<param>(bytes));
    txt << pheader() << std::endl;
    txt << to_string(p) << std::endl << std::endl;
    txt << sheader(mask) << std::endl; 
    int n = param::size;
    while (n < fsize)
    {
        state s = from_binary(wrap<state>(bytes + n));
        txt << to_string(s, mask) << std::endl;
        n += state::size;
    }
    delete[] bytes;
}

