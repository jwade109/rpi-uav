#include <string.h>
#include <signal.h>
#include <stdlib.h>
#include <SerialIMU.h>

SerialIMU::SerialIMU()
{
    Message m;
    memset(&m, 0, sizeof(Message));
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

    in.open("/dev/ttyACM0");
    if (!in) return 1;

    mem = create_shared_memory(sizeof(Message) + 1);
    memset(mem, 0, sizeof(Message) + 1);

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
            memset(mem, 0, sizeof(Message) + 1);
            Message m = parseMessage();
            memcpy(mem + 1, &m, sizeof(m));
            mem[0] = 1;
            memset(buffer, 0, MSG_LEN);
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

Message SerialIMU::get()
{
    if (mem[0]) last = *((Message*)(mem + 1));
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

Message SerialIMU::parseMessage()
{
    char* cursor;
    Message data;
    data.millis = strtol(buffer, &cursor, 10);
    data.heading = strtod(cursor, &cursor);
    data.pitch = strtod(cursor, &cursor);
    data.roll = strtod(cursor, &cursor);
    data.calib = strtol(cursor, &cursor, 10);
    data.alt = strtod(cursor, &cursor);
    return data;
}
