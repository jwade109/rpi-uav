#include <iostream>
#include <iomanip>
#include <string>

#include <uav/logging>
#include <uav/control>

int main(int argc, char** argv)
{
    auto archives = argc > 1 ? uav::restore_sorted(argv[1]) :
                               uav::restore_sorted();

    if (archives.size() == 0)
    {
        std::cerr << "Invalid filename." << std::endl;
        return 1;
    }
    if (archives["Param"].size() == 0)
    {
        std::cerr << "No parameter info." << std::endl;
        return 2;
    }
    if (archives["Param"].size() > 1)
    {
        std::cerr << "Ambiguous: multiple parameter packets." << std::endl;
        return 3;
    }
    if (archives["State"].size() == 0)
    {
        std::cerr << "No state info." << std::endl;
        return 4;
    }


    std::string outfile = argc > 2 ? argv[2] : "log/out.txt";

    std::ofstream txt(outfile, std::ios::out | std::ios::binary);
    if (!txt)
    {
        std::cerr << "Could not create text file." << std::endl;
        return 3;
    }

    txt << uav::state::header() << std::endl;
    for (auto e : archives["State"])
    {
        uav::state s;
        e >> s;
        txt << s << std::endl;
    }
    return 0;
}

