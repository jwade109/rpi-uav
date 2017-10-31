#include "logstream.h"

#include <iostream>
#include <iomanip>
#include <string>
#include <cmath>
#include <sstream>
#include <chrono>

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

bool overwrite = true;
std::string outfile = "log/data.bin";
std::deque<uav::archive> archives;

namespace uav
{

logstream debug("Debug"), info("Info"), error("Error");

logstream::logstream(const std::string& streamname) :
    _name(streamname) { }

const std::string& logstream::name() const
{
    return _name;
}

logstream& logstream::operator << (const char* c)
{
    return *this << std::string(c);
}

logstream& logstream::operator << (const std::string& s)
{
    auto pos = s.find('\n');
    if (pos == std::string::npos)
    {
        _buffer << s;
        return *this;
    }
    _buffer << s.substr(0,pos);
    archive out;
    out << _buffer.str();
    _buffer.str("");
    *this << out;
    return *this << s.substr(pos+1);
}

logstream& logstream::operator << (archive& a)
{
    a.name() = _name;
    archives.push_back(a);
}

void reset()
{
    archives.clear();
}

void flush(const std::string& filename)
{
    auto flags = std::ios::out | std::ios::binary | std::ios::app;
    if (overwrite) { overwrite = false; flags &= ~std::ios::app; }
    std::ofstream data(filename, flags);
    while (!archives.empty())
    {
        std::vector<uint8_t> raw = archives.front().raw();
        char* wptr = reinterpret_cast<char*>(&raw[0]);
        data.write(wptr, raw.size());
        archives.pop_front();
    }
    data.flush();
    data.close();
}

std::vector<archive> restore(const std::string& filename)
{
    std::ifstream infile(filename, std::ios::in | std::ios::binary);
    if (!infile) return std::vector<archive>();

    auto fsize = infile.tellg();
    infile.seekg(0, std::ios::end);
    fsize = infile.tellg() - fsize;
    infile.seekg(0, std::ios::beg);

    std::vector<archive> archives;
    std::vector<uint8_t> bytes(fsize, 0);
    infile.read(reinterpret_cast<char*>(bytes.data()), fsize);

    size_t n = 0;
    while (n < fsize)
    {
        auto a = archive::package(bytes.data() + n);
        n += a.size();
        archives.push_back(a);
    }
    return archives;
}

} // namespace uav
