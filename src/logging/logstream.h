#ifndef LOGSTREAM_H
#define LOGSTREAM_H

#include <fstream>
#include <sstream>
#include <deque>

#include "data_frame.h"

namespace uav
{

void reset();

void flush();

class logstream
{
    std::stringstream ss;
    const std::string name;

    public:

    logstream(const std::string& name);

    void add(std::vector<uint8_t> data);

    template <typename T> logstream& operator << (T object)
    {
        add(object.bytes);
        return *this;
    }
};

extern logstream debug, info, error;

} // namespace uav

#endif // LOGSTREAM_H
