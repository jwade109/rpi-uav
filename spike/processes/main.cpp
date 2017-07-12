#include <stdio.h>
#include <string.h>
#include <sys/types.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/mman.h>
#include <string.h>
#include <stdint.h>
#include <inttypes.h>
#include <fstream>
#include <stdbool.h>
#include <TimeUtil.h>

#include <Message.h>

#define MSG_LEN 100

void* create_shared_memory(size_t size) {
    // Memory buffer will be readable and writable:
    int protection = PROT_READ | PROT_WRITE;

    // The buffer will be shared (meaning other processes can access it), but
    // anonymous (meaning third-party processes cannot obtain an address for it),
    // so only this process and its children will be able to use it:
    int visibility = MAP_ANONYMOUS | MAP_SHARED;

    // The remaining parameters to `mmap()` are not important for this use case,
    // but the manpage for `mmap` explains their purpose.
    return mmap(NULL, size, protection, visibility, 0, 0);
}

message_t parseMessage(char* buffer)
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

void printMessage(message_t &msg)
{
    printf("%"PRIu64", %lf, %lf, %lf, %d, %lf\n",
        msg.millis, msg.heading, msg.pitch, msg.roll,
        msg.calib, msg.alt);
}

int main(int argc, char** argv)
{
    char* shmem = (char*) create_shared_memory(sizeof(message_t) + 1);
    memset(shmem, 0, sizeof(message_t) + 1);

    std::ifstream f;
    f.open("/dev/ttyACM0");
    if (!f)
    {
        printf("Could not open serial port.\n");
        return 1;
    }
    printf("Opened /dev/ttyACM0\n");

    // *shmem = 1;
    int pid = fork();
    if (pid > 0)
    {
        for(;;)
        {
            bool read = *shmem;
            if (read)
            {
                uint64_t time = getUnixTime(MILLI);
                printf("[%"PRIu64"] ", time);
                message_t m = *((message_t*)(shmem + 1));
                printf("(%"PRIu64") ", (time - m.millis) % 1000);
                printMessage(m);
                waitFor(20, MILLI);
            }
        }
    }

    size_t ptr = 0;
    bool begin = false;
    bool message = false;
    char ch;
    char buffer[MSG_LEN];
    bool debug = argc > 1;

    while (f.get(ch))
    {
        if (ch == '#') begin = true;
        if (ch == '!') begin = false;
        if (ch == '<' && begin)
        {
            message = true;
            f.get(ch);
        }
        else if (ch == '>' && begin)
        {
            if (debug)
                printf("[%"PRIu64"] (Updating...)\n", getUnixTime(MICRO));
            message = false;
            memset(shmem, 0, sizeof(message_t) + 1);
            message_t m = parseMessage(buffer);
            memcpy(shmem + 1, &m, sizeof(m));
            memset(buffer, 0, MSG_LEN);
            *shmem = 1;
            ptr = 0;
            if (debug)
                printf("[%"PRIu64"] (Done!)\n", getUnixTime(MICRO));
        }
        if (message && begin)
        {
            buffer[ptr] = ch;
            ++ptr;
        }
    }
    return 0;
}
