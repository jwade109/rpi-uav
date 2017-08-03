#include <string.h>
#include <signal.h>
#include <stdlib.h>
#include <ardimu.h>
#include <unistd.h>
#include <smem.h>

Arduino::Arduino()
{
    Message m;
    memset(&m, 0, sizeof(Message));
    last = m;
    child_pid = -1;
}

Arduino::~Arduino()
{
    if (child_pid > 0) kill(child_pid, SIGKILL);
    in.close();
}

int Arduino::begin()
{
    if (child_pid != -1)
    {
        fprintf(stderr, "Arduino: Child process already exists\n");
        return 1;
    }

    //                 | 1 byte | 1 byte | sizeof(Message) |
    // memory mapping: | ON/OFF | RD/WRT | MESSAGE ------> |
    mem = (char*) sharedmem(sizeof(Message) + 2);
    memset(mem, 0, sizeof(Message) + 2);
    
    in.open("/dev/ttyACM0");
    if (!in)
    {
        fprintf(stderr, "Arduino: Could not open /dev/ttyACM0\n");
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
            fprintf(stderr, "Arduino: Arduino reporting error: "
                    "\"%s\"\n", buffer);
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

Message Arduino::get()
{
    if (mem[0] == 1 && mem[1]) last = *((Message*)(mem + 2));
    return last;
}

Message Arduino::parseMessage(char* buffer)
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
