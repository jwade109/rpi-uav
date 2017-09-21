#include <iostream>
#include <fstream>
#include <string>
#include <sstream>
#include <vector>
#include <tuple>
#include <signal.h>
#include <ncurses.h>
#include <algorithm>

#include <uavcore.h>

void sigint(int signal)
{
    endwin();
}

int main(int argc, char** argv)
{
    using namespace uav;

    // signal(SIGINT, sigint);

    std::string data_fn = argc > 1 ? argv[1] : "log/data.bin";
    std::string events_fn = argc > 2 ? argv[2] : "log/events.txt";

    std::ifstream data(data_fn, std::ios::binary | std::ios::in),
                  events(events_fn, std::ios::in);
    if (!data)
    {
        std::cout << "Invalid filename: " << data_fn << std::endl;
        return 1;
    }
    if (!events)
    {
        std::cout << "Invalid filename: " << events_fn << std::endl;
        return 2;
    }

    enum fileflag : bool { dataf = 0, eventf };
    using entry = std::tuple<std::uint64_t, std::string, fileflag>;
    std::vector<entry> entries;

    std::string line;
    while (getline(events, line))
    {
        std::stringstream ss(line);
        int count = 0;
        char ch;
        while (count < 2) { ss >> ch; count += ch == '['; }
        uint64_t temp, timestamp;
        ss >> timestamp;
        ss >> ch; ss >> temp;
        timestamp = timestamp * 1000 + temp;
        ss >> ch; ss >> temp;
        timestamp = timestamp * 1000 + temp;
        entries.push_back(std::make_tuple(timestamp, line, eventf));
    }

    std::streampos fsize = data.tellg();
    data.seekg(0, std::ios::end);
    fsize = data.tellg() - fsize;
    data.seekg(0, std::ios::beg);

    const uint64_t mask = -1;
    char* bytes = new char[fsize];
    data.read(bytes, fsize);
    int n = param::size;
    while (n < fsize)
    {
        state s = from_binary(wrap<state>(bytes + n));
        entries.push_back(std::make_tuple(s.t_abs * 1000,
                    to_string(s, mask), dataf));
        n += state::size;
    }
    delete[] bytes;

    auto comp = [] (entry& a, entry& b)
        { return std::get<0>(a) < std::get<0>(b); };
    std::sort(begin(entries), end(entries), comp);

    for (auto e : entries)
        std::cout << std::get<1>(e).substr(0,70) << std::endl;

    return 0;
}

