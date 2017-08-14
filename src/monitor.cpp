#include <iostream>
#include <fstream>
#include <chrono>
#include <string>
#include <malloc.h>
#include <assert.h>

#include <monitor.h>

const size_t sl = 1000*1000, pl = 20, el = 10*1000;
std::string sbuf[sl], pbuf[pl], ebuf[el];

namespace uav
{
    Monitor::Monitor():
        states(sbuf, sl),
        params(pbuf, pl),
        events(ebuf, el)
    {
    }

    Monitor::~Monitor()
    {
        flush();
        close();
    }

    int Monitor::open(bool append)
    {
        auto flags = std::ios::out | std::ios::binary;
        if (append) flags |= std::ios::app;

        fstates.open("log/states.txt", flags);
        fparams.open("log/params.txt", flags);
        fevents.open("log/events.txt", flags);

        return 0;
    }

    void Monitor::flush()
    {
        while (states.available())
        {
            fstates << states.get();
        }
        while (params.available())
        {
            fparams << params.get();
        }
        while (events.available())
        {
            fevents << events.get();
        }

        fstates.flush();
        fparams.flush();
        fevents.flush();
    }

    void Monitor::close()
    {
        fstates.close();
        fparams.close();
        fevents.close();
    }
}
