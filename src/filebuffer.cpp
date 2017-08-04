#include <filebuffer.h>
#include <signal.h>
#include <unistd.h>
#include <stdio.h>
#include <smem.h>

const uint16_t buffer_len = 500;

FileBuffer::FileBuffer(const std::string fn) : filename(fn), child_pid(-1)
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

FileBuffer::~FileBuffer()
{
    if (child_pid > 0) kill(child_pid, SIGKILL);
    fclose(out);
}

int FileBuffer::begin()
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

void FileBuffer::push(const std::string str)
{
    for (size_t i = 0; i < str.length(); i++)
    {
        put(str[i]);
    }
}

void FileBuffer::push(const char* array, uint16_t len)
{
    for (uint16_t i = 0; i < len; i++)
    {
        put(array[i]);
    }
}

void FileBuffer::flush()
{
    while (available() > 0)
    {
        char c = get();
        fprintf(out, "%c", c);
    }
    fflush(out);
}

void FileBuffer::initbuffer()
{
    *size() = buffer_len;
    *start() = 0;
    *end() = 0;
}

uint16_t FileBuffer::available()
{
    return (uint16_t) (*size() + *end() - *start()) % *size();
}

void FileBuffer::put(char c)
{
    if (full())
    {
        fprintf(stderr, "Buffer full!\n");
        return;
    }
    buf[(*end())] = c;
    *end() = (*end() + 1) % *size();
}

bool FileBuffer::full()
{
    return (*end() + 1) % *size() == *start();
}

char FileBuffer::get()
{
    char ret = buf[(*start())];
    *start() = (*start() + 1) % *size();
    return ret;
}

uint16_t* FileBuffer::size()
{
    return dat;
}

uint16_t* FileBuffer::start()
{
    return dat + 1;
}

uint16_t* FileBuffer::end()
{
    return dat + 2;
}
