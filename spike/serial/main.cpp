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

void parseMessage(char buffer[1024]);

void printData();

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
            parseMessage(buffer);
            printData();
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

void printData()
{
    printf("%"PRIu64" %lf %lf %lf %02x %lf %lf\n",
        data.millis, data.heading, data.pitch, data.roll,
        data.calib, data.altA, data.altB);
}

void parseMessage(char buffer[1024])
{
    char* cursor;
    data.millis = strtol(buffer, &cursor, 10);
    data.heading = strtod(cursor, &cursor);
    data.pitch = strtod(cursor, &cursor);
    data.roll = strtod(cursor, &cursor);
    data.calib = strtol(cursor, &cursor, 10);
    data.altA = strtod(cursor, &cursor);
    data.altB = strtod(cursor, NULL);
}
