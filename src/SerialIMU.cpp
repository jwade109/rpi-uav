#include <string.h>
#include <signal.h>
#include <stdlib.h>
#include <SerialIMU.h>

SerialIMU::SerialIMU()
{
    message_t m;
    memset(&m, 0, sizeof(message_t));
    last = m;
    child_pid = -1;
}

SerialIMU::~SerialIMU()
{
    if (child_pid > 0) kill(child_pid, SIGKILL);
    in.close();
}

int SerialIMU::begin()
{
    if (child_pid != -1) return 2;

    mem = (char*) create_shared_memory(sizeof(message_t) + 1);
    memset(mem, 0, sizeof(message_t) + 1);
    in.open("/dev/ttyACM0");
    if (!in)
    {
        return 1;
    }

    int pid = fork();
    if (pid > 0)
    {
        child_pid = pid;
        return 0;
    }

    size_t ptr = 0;
    bool begin = false;
    bool message = false;
    char ch;
    char buffer[MSG_LEN];
    int cnt = 0;

    while (in.get(ch))
    {
        if (ch == '#') begin = true;
        if (ch == '!') begin = false;
        if (ch == '<' && begin)
        {
            message = true;
            in.get(ch);
        }
        else if (ch == '>' && begin)
        {
            message = false;
            memset(mem, 0, sizeof(message_t) + 1);
            message_t m = parseMessage(buffer);
            memcpy(mem + 1, &m, sizeof(m));
            memset(buffer, 0, MSG_LEN);
            *mem = 1;
            ptr = 0;
        }
        if (message && begin)
        {
            buffer[ptr] = ch;
            ++ptr;
        }
    }
    return 0;
}

message_t SerialIMU::get()
{
    int read = *mem;
    if (read)
    {
        message_t m = *((message_t*)(mem + 1));
        last = m;
        return m;
    }
    return last;
}

char* SerialIMU::create_shared_memory(size_t size)
{
    // Memory buffer will be readable and writable:
    int protection = PROT_READ | PROT_WRITE;

    // The buffer will be shared (meaning other processes can access it), but
    // anonymous (meaning third-party processes cannot obtain an address for it),
    // so only this process and its children will be able to use it:
    int visibility = MAP_ANONYMOUS | MAP_SHARED;

    // The remaining parameters to `mmap()` are not important for this use case,
    // but the manpage for `mmap` explains their purpose.
    return (char*) mmap(NULL, size, protection, visibility, 0, 0);
}

message_t SerialIMU::parseMessage(char* buffer)
{
    char* cursor;
    message_t data;
    data.millis = strtol(buffer, &cursor, 10);
    data.heading = strtod(cursor, &cursor);
    data.pitch = strtod(cursor, &cursor);
    data.roll = strtod(cursor, &cursor);
    data.calib = strtol(cursor, &cursor, 10);
    data.alt = strtod(cursor, &cursor);
    return data;
}

void SerialIMU::printMessage(message_t &msg)
{
    printf("%" PRIu64 ", %lf, %lf, %lf, %d, %lf\n",
        msg.millis, msg.heading, msg.pitch, msg.roll,
        msg.calib, msg.alt);
}
