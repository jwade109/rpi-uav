#ifndef FILE_BUFFER_H
#define FILE_BUFFER_H

#include <string>
#include <stdint.h>

class FileBuffer
{
    public:

    FileBuffer(const std::string fn, uint16_t len);
    ~FileBuffer();

    int begin();

    void push(const std::string str);
    void push(const char* array, uint16_t len);
    void flush();

    private:

    int child_pid;
    std::string filename;
    FILE* out;
    char* buf;
    uint16_t* dat;
    uint16_t buffer_len;

    void initbuffer();

    uint16_t available();
    void put(char c);
    bool full();
    char get();

    uint16_t* size();
    uint16_t* start();
    uint16_t* end();
};

#endif // FILE_BUFFER_H
