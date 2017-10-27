#ifndef LOGSTREAM_H
#define LOGSTREAM_H

#include <fstream>
#include <sstream>
#include <deque>

#include "uavcore.h"

namespace uav
{

void reset();

void include(param p);

void include(state s);

void flush();

class logstream
{
    std::stringstream ss;
    const std::string name;

    public:

    logstream(const std::string& name);

    template <typename T>
    logstream& operator << (const T& t)
    {
        ss << t;
        return *this;
    }

    void operator () (const std::string& s);

    logstream& operator << (std::ostream& (*)(std::ostream&));
};

extern logstream debug, info, error;

} // namespace uav

#endif // LOGSTREAM_H
