#ifndef MONITOR_H
#define MONITOR_H

#include <iostream>
#include <fstream>

namespace uav
{
    namespace log
    {
        extern std::ofstream states;
        extern std::ofstream params;
        extern std::ofstream events;

        void open();
        void flush();
        void close();
    }
}

#endif // MONITOR_H
