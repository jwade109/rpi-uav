#include "logstream.h"

#include <iostream>
#include <iomanip>
#include <string>
#include <cmath>
#include <sstream>
#include <chrono>

#include <uav/math>
#include "uavcore.h"
#include "serial.h"

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

param paramlog;
std::deque<state> statelog;
std::deque<std::string> textlog;

logstream debug("DEBUG"), info("INFO"), error("ERROR");

logstream::logstream(const std::string& streamname) :
    name(streamname) { }

logstream& logstream::operator << (std::ostream& (*)(std::ostream& os))
{
    textlog.push_back("[" + name + "]\t" + timestamp() + ss.str());
    ss.str(std::string());
    return *this;
}

void logstream::operator () (const std::string& s)
{
    textlog.push_back("[" + name + "]\t" + timestamp() + s);
}

void reset()
{
    paramlog = {0};
    statelog.clear();
    textlog.clear();
}

void include(param p)
{
    paramlog = p;
}

void include(state s)
{
    statelog.push_back(s);
}

void flush()
{
    std::ofstream
        text("log/events.txt", std::ios::out),
        data("log/data.bin", std::ios::out | std::ios::binary);

    {
        param::bin b = serialize(paramlog);
        data.write(reinterpret_cast<const char*>(b.data()), param::size);
    }
    while (!statelog.empty())
    {
        state::bin b = serialize(statelog.front());
        statelog.pop_front();
        data.write(reinterpret_cast<const char*>(b.data()), state::size);
    }
    while (!textlog.empty())
    {
        text << textlog.front() << "\n";
        textlog.pop_front();
    }
}

} // namespace uav
