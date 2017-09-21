#include <iostream>
#include <fstream>
#include <string>
#include <sstream>
#include <vector>
#include <tuple>
#include <signal.h>
#include <ncurses.h>
#include <algorithm>
#include <assert.h>

#include <uavcore.h>

int main(int argc, char** argv)
{
    using namespace uav;

    std::string data_fn = argc > 1 ? argv[1] : "../../log/data.bin";
    std::string events_fn = argc > 2 ? argv[2] : "../../log/events.txt";

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

    initscr();
    raw();
    keypad(stdscr, true);
    noecho();
    cbreak();

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
                    to_string(s, 0b11111111), dataf));
        n += state::size;
    }
    delete[] bytes;

    auto comp = [] (entry& a, entry& b)
        { return std::get<0>(a) < std::get<0>(b); };
    std::sort(begin(entries), end(entries), comp);

    assert(entries.size() > 0);

    uint16_t ch = 0;
    int64_t pos = 0;
    while (ch != 3)
    {
        move(0,0);
        clrtoeol();
        move(1,0);
        clrtoeol();
        move(2,0);
        clrtoeol();
        move(3,0);
        clrtoeol();

        mvprintw(0,0,"Key pressed: ");
        attron(A_BOLD);
        printw("0x%02x (%c)", ch, ch);
        attroff(A_BOLD);

        uint16_t key[] = {KEY_UP, KEY_DOWN, KEY_RIGHT, KEY_LEFT};
        const int keydir[] = {-1, 1, 0, 0};
        const char *keystr[] = {"UP", "DOWN", "RIGHT", "LEFT"};
        
        for (int i = 0; i < 4; i++)
        {
            if (key[i] == ch)
            {
                printw("%s", keystr[i]);
                pos += keydir[i];
                if (pos < 0) pos = 0;
                uint64_t max = entries.size();
                if (pos + 1 > max) pos = max - 1;
                const char* str = std::get<1>(entries[pos]).c_str();
                mvprintw(1, 0, "%lld/%llu: %s", pos, max, str);
            }
        }

        refresh();
        ch = getch();
    }

    getch();
    endwin();
    return 0;
}

