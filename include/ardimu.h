#ifndef ARDIMU_H
#define ARDIMU_H

#include <fstream>
#include <inttypes.h>

typedef struct
{
    uint64_t millis;
    float heading;
    float pitch;
    float roll;
    uint8_t calib;
    float alt;
}
Message;

#define MSG_LEN 100

class Arduino 
{
    public:

    Message last;

    Arduino();
    ~Arduino();

    int begin();
    void get(float& h, float& p, float& r, float& z, uint8_t& cal);
    Message get();

    private:

    char buffer[MSG_LEN];
    std::ifstream in;
    char* mem;
    int child_pid;

    Message parseMessage(char* buffer);
};

#endif // ARDIMU_H
