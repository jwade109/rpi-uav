#include <stdio.h>
#include <iostream>
#include <fstream>
#include <stdint.h>
#include <inttypes.h>
#include <stdbool.h>
#include <string>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <TimeUtil.h>

#define MSG_LEN 100

using namespace std;

typedef struct
{
    uint64_t millis;
    double heading;
    double pitch;
    double roll;
    uint8_t calib;
    double alt;
}
message_t;

char nodename[20] = {0};
char buffer[MSG_LEN];
int  pip[2];

void parseMessage(char buffer[MSG_LEN], message_t &data)
{
    char* cursor;
    data.millis = strtol(buffer, &cursor, 10);
    data.heading = strtod(cursor, &cursor);
    data.pitch = strtod(cursor, &cursor);
    data.roll = strtod(cursor, &cursor);
    data.calib = strtol(cursor, &cursor, 10);
    data.alt = strtod(cursor, &cursor);
}

void printData(message_t &data)
{
    printf("%"PRIu64" %lf %lf %lf %02x %lf\n",
        data.millis, data.heading, data.pitch, data.roll,
        data.calib, data.alt);
}

int listenForSerial(int argc)
{
    sprintf(nodename, "[%d Reciever]", getpid());
    for(;;)
    {
        read(pip[0], buffer, MSG_LEN);
        message_t msg;
        parseMessage(buffer, msg);
        printf("%s ", nodename);
        printData(msg);
    }
    return 0;
}

int main(int argc, char** argv)
{
    int res = pipe(pip);
    if (res < 0) return 1;
    res = fork();
    // child
    if (res == 0) return listenForSerial(argc);

    sprintf(nodename, "[%d Reader]", res);
    bool debug = argc > 1;
    bool begin = false;
    bool message = false;
    char ch;
    ifstream f;
    f.open("/dev/ttyACM0");
    if (!f)
    {
        printf("%s Could not open serial port.\n", nodename);
        return 1;
    }
    printf("%s Opened /dev/ttyACM0\n", nodename);
    size_t ptr = 0;

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
            message = false;
            if (debug) printf("%s (%s)\n", nodename, buffer);
            write(pip[1], buffer, MSG_LEN);
            memset(buffer, 0, MSG_LEN);
            ptr = 0;
        }
        if (message && begin)
        {
            if (ch == 0) ch = ' ';
            buffer[ptr] = ch;
            ++ptr;
        }
    }
    return 0;
}
