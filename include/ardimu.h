#ifndef ARDIMU_H
#define ARDIMU_H

#include <fstream>
#include <message.h>

#define MSG_LEN 100

class Arduino 
{
    public:

    Message last;

    Arduino();
    ~Arduino();

    int begin();
    Message get();

    private:

    char buffer[MSG_LEN];
    std::ifstream in;
    char* mem;
    int child_pid;

    Message parseMessage(char* buffer);
};

#endif // ARDIMU_H