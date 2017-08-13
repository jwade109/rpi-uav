#include <iostream>
#include <fstream>
#include <chrono>
#include <ctime>
#include <string>

#include <monitor.h>

namespace uav
{
    namespace log
    {
        std::ofstream states;

        std::ofstream params;

        std::ofstream events;

        void open()
        {
            states.open("log/states.txt", std::ios::out);
            params.open("log/params.txt", std::ios::out);
            events.open("log/events.txt", std::ios::out);
        }

        void flush()
        {
            states.flush();
            params.flush();
            events.flush();
        }

        void close()
        {
            states.close();
            params.close();
            events.close();
        }
    }
}
