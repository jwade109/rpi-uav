#ifndef FILE_BUFFER_H
#define FILE_BUFFER_H

#include <string>
#include <signal.h>
#include <stdint.h>
#include <unistd.h>
#include <stdio.h>
#include <smem.h>

const uint16_t buffer_len = 500;

class FileBuffer
{
public:

    FileBuffer(const std::string fn) : filename(fn), child_pid(-1)
    {
        out = fopen(filename.c_str(), "w");
        if (out == 0)
        {
            fprintf(stderr, "FileBuffer: failed to open %s\n",
                    filename.c_str());
        }
        dat = (uint16_t*) sharedmem(sizeof(uint16_t) * 3);
        buf = (char*) sharedmem(buffer_len);
        if (buf == 0 || dat == 0)
        {
            fprintf(stderr, "FileBuffer: failed to"
                    "allocate memory\n");
        }
        initbuffer();
    }

    ~FileBuffer()
    {
        if (child_pid > 0) kill(child_pid, SIGKILL);
        fclose(out);
    }

    int begin()
    {
        if (child_pid != -1)
        {
            fprintf(stderr, "FileBuffer: child process already exists\n");
            return 1;
        }
        
        int pid = fork();
        if (pid > 0)
        {
            child_pid = pid;
            return 0;
        }

        while (1) write();
        return 0;
    }

    void push(const std::string str)
    {
        for (size_t i = 0; i < str.length(); i++)
        {
            put(str[i]);
        }
    }

    void push(const char* array, uint16_t len)
    {
        for (uint16_t i = 0; i < len; i++)
        {
            put(array[i]);
        }
    }

    void write()
    {
        while (available() > 0)
        {
            char c = get();
            fprintf(out, "%c", c);
        }
        fflush(out);
    }

    uint16_t getsize()
    {
        return *size();
    }

    uint16_t getstart()
    {
        return *start();
    }

    uint16_t getend()
    {
        return *end();
    }

private:

    int child_pid;
    std::string filename;
    FILE* out;
    char* buf;
    uint16_t* dat;

    void initbuffer()
    {
        *size() = buffer_len;
        *start() = 0;
        *end() = 0;
    }

    uint16_t available()
    {
        return (uint16_t) (*size() + *end() - *start()) % *size();
    }

    void put(char c)
    {
        if (full())
        {
            fprintf(stderr, "Buffer full!\n");
            return;
        }
        buf[(*end())] = c;
        *end() = (*end() + 1) % *size();
    }

    bool full()
    {
        return (*end() + 1) % *size() == *start();
    }

    char get()
    {
        char ret = buf[(*start())];
        *start() = (*start() + 1) % *size();
        return ret;
    }

    uint16_t* size()
    {
        return dat;
    }

    uint16_t* start()
    {
        return dat + 1;
    }

    uint16_t* end()
    {
        return dat + 2;
    }
};

#endif // FILE_BUFFER_H
