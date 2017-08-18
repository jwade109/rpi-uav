#ifndef MONITOR_H
#define MONITOR_H

#include <fstream>

#include <control.h>

/*
Embedded Tool Kit
Copyright (C) 2015 Samuel Cowen

Permission is hereby granted, free of charge, to any person obtaining
a copy of this software and associated documentation files (the
"Software"), to deal in the Software without restriction, including
without limitation the rights to use, copy, modify, merge, publish,
distribute, sublicense, and/or sell copies of the Software, and to
permit persons to whom the Software is furnished to do so, subject to
the following conditions:

The above copyright notice and this permission notice shall be
included in all copies or substantial portions of the Software.
*/

// modified from original by Wade Foster on August 13th 2017

namespace etk
{
    template <class T, bool overwrite = false> class RingBuffer
    {
        public:

        RingBuffer(T* buffer, size_t sz)
        {
            size = sz;
            start = 0;
            end = 0;
            length = 0;
            buf = buffer;
        }

        bool is_full()
        {
            return size == length;
        }

        size_t available()
        {
            return length;
        }

        void put(T b)
        {
            if(!overwrite)
            {
                if(is_full())
                    return;
            }
            buf[end] = b;
            end = (end + 1) % size;
            if (is_full())
            {
                start = (start + 1) % size;
            }
            else
            {
                length++;
            }
        }

        void increment()
        {
            end = (end + 1) % size;
        }

        T get()
        {
            T ret = buf[start];
            start = (start + 1) % size;
            length--;
            return ret;
        }

        T peek_ahead(size_t n=0)
        {
            size_t pos = (start+n) % size;
            return buf[pos];
        }

        void empty()
        {
            start = 0;
            end = 0;
            length = 0;
        }

        std::string tostring()
        {
            return "Start: " + std::to_string(start) +
                   " End: " + std::to_string(end) +
                   " Length: " + std::to_string(length);
        }

        private:

        size_t size;
        size_t start;
        size_t end;
        size_t length;
        T* buf;
    };
}

namespace uav
{
    const size_t statefields = 21;  // number of fields in each
    const size_t paramfields = 23;

    const size_t statelen = 63;     // number of bytes of each
    const size_t paramlen = 171;    // respective member

    namespace log
    {
        extern uav::Param param;
        extern etk::RingBuffer<uav::State, true> states;
        extern etk::RingBuffer<std::string, true> events;

        int open(bool append = false);
        void flush();
        void close();
    }

    int tobuffer(Param& prm, char* buffer);

    int tobuffer(State& it, char* buffer);

    int frombuffer(Param& prm, char* buffer);

    int frombuffer(State& it, char* buffer);

    std::string pheader();

    std::string sheader(uint64_t mask);

    std::string to_string(Param prm);

    std::string to_string(State it, uint64_t mask);
}


#endif // MONITOR_H
