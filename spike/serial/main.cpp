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

using namespace std;

typedef struct
{
    uint64_t millis;
    double heading;
    double pitch;
    double roll;
    uint8_t calib;
    double altA;
    double altB;
}
message_t;

message_t data;

void parseMessage(char buffer[1024], message_t &msg);

int main(int argc, char** argv)
{
    bool debug = argc > 1;
    bool begin = false;
    bool message = false;
    char ch;
    ifstream f;
    f.open("/dev/ttyACM0");
    if (!f)
    {
        cout << "Could not open serial port." << endl;
        return 1;
    }
    char buffer[1024] = {0};
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
            if (debug) printf("(%s)\n", buffer);
            parseMessage(buffer, data);
            memset(buffer, 0, 1024);
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

void parseMessage(char buffer[1024], message_t &msg)
{
    memset(&msg, 0, sizeof(msg));
    char* cursor;
    msg.millis = strtol(buffer, &cursor, 10);
    msg.heading = strtod(cursor, &cursor);
    msg.pitch = strtod(cursor, &cursor);
    msg.roll = strtod(cursor, &cursor);
    msg.calib = strtol(cursor, &cursor, 10);
    msg.altA = strtod(cursor, &cursor);
    msg.altB = strtod(cursor, NULL);
    printf("%"PRIu64" %lf %lf %lf %02x %lf %lf\n",
        msg.millis, msg.heading, msg.pitch, msg.roll,
        msg.calib, msg.altA, msg.altB);
}
