#include "logstream.h"

#include <iostream>
#include <iomanip>
#include <string>
#include <cmath>
#include <sstream>
#include <chrono>

namespace uav
{

std::string timestamp()
{
    using namespace std;
    using namespace std::chrono;
    uint64_t time = duration_cast<microseconds>(steady_clock::now()
                  .time_since_epoch()).count();
    std::stringstream s;
    s.setf(ios::fixed);

    uint64_t sec = (time/1000000);
    uint64_t milli = (time/1000) % 1000;
    uint64_t micro = time % 1000;

    s << "[" << setw(6) << setfill('0') << sec << "."
        << setw(3) << setfill('0') << milli << "."
        << setw(3) << setfill('0') << micro << "] ";
    return s.str();
}

std::string outfile = "log/data.bin";
std::deque<data_frame> frames;

logstream debug("DEBUG"), info("INFO"), error("ERROR");

logstream::logstream(const std::string& streamname) :
    name(streamname) { }

void logstream::add(std::vector<uint8_t> data)
{
    frames.push_back(data_frame(name, data));
}

void reset()
{
    frames.clear();
}

void flush()
{
    std::ofstream data(outfile, std::ios::out | std::ios::binary);
    while (!frames.empty())
    {
        std::vector<uint8_t> raw = frames.front().raw();
        char* wptr = reinterpret_cast<char*>(&raw[0]);
        data.write(wptr, raw.size());
        frames.pop_front();
    }
    data.flush();
    data.close();
}

} // namespace uav
