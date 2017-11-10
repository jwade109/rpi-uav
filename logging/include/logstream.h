#ifndef LOGSTREAM_H
#define LOGSTREAM_H

#include <fstream>
#include <sstream>
#include <deque>
#include <map>

#include "archive.h"

namespace uav
{

void reset();

void flush(const std::string& fn = "log/data.bin");

std::vector<archive> restore(const std::string& fn = "log/data.bin");

std::map<std::string, std::vector<archive>>
    restore_sorted(const std::string& fn = "log/data.bin");

class logstream
{
    public:

    logstream(const std::string& name);

    const std::string& name() const;

    template <typename T>
    logstream& operator << (T x)
    {
        return *this << std::to_string(x);
    }

    logstream& operator << (const char* c);
    logstream& operator << (const std::string& a);
    logstream& operator << (archive& a);

    private:

    const std::string _name;
    std::stringstream _buffer;
};

extern logstream debug, info, error;

} // namespace uav

#endif // LOGSTREAM_H
