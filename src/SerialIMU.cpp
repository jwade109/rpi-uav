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
    if (child_pid != -1)
    {
        fprintf(stderr, "SerialIMU: Child process already exists\n");
        return 1;
    }

    //                 | 1 byte | 1 byte | sizeof(Message) |
    // memory mapping: | ON/OFF | RD/WRT | MESSAGE ------> |
    mem = (char*) create_shared_memory(sizeof(Message) + 2);
    memset(mem, 0, sizeof(Message) + 2);
    
    in.open("/dev/ttyACM0");
    if (!in)
    {
        fprintf(stderr, "SerialIMU: Could not open /dev/ttyACM0\n");
        return 2;
    }

    int pid = fork();
    if (pid > 0)
    {
        child_pid = pid;
        while(mem[0] == 0);
        return mem[0] == 1 ? 0 : 3;
    }

    size_t ptr = 0;
    mem[0] = 0;
    bool message = false;
    char ch;
    char buffer[MSG_LEN];

    while (in.get(ch))
    {
        if (ch == '#') mem[0] = 1;
        else if (ch == '!')
        {
            mem[0] = 2;
            memset(buffer, 0, MSG_LEN);
            buffer[0] = '!';
            in.get(ch);
            buffer[1] = ch;
            for (int i = 2; i < MSG_LEN && ch != '!'; i++)
            {
                in.get(ch);
                buffer[i] = ch;
            }
            fprintf(stderr, "SerialIMU: Arduino reporting error: \"%s\"\n", buffer);
            while (1);
        }
        else if (ch == '<' && mem[0] == 1)
        {
            message = true;
            in.get(ch);
        }
        else if (ch == '>' && mem[0] == 1)
        {
            message = false;
            memset(mem + 1, 0, sizeof(Message) + 1);
            Message m = parseMessage(buffer);
            memcpy(mem + 2, &m, sizeof(m));
            mem[1] = 1;
            memset(buffer, 0, MSG_LEN);
            ptr = 0;
        }

        if (message && mem[0] == 1)
        {
            buffer[ptr] = ch;
            ++ptr;
        }
    }
    return 0;
}

Message SerialIMU::get()
{
    if (mem[0] == 1 && mem[1]) last = *((Message*)(mem + 2));
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

Message SerialIMU::parseMessage(char* buffer)
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
