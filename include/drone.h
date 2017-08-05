#ifndef DRONE_H
#define DRONE_H

#include <stdint.h>
#include <pid.h>
#include <filebuffer.h>
#include <ardimu.h>

typedef struct Iter
{

}

typedef struct Param
{

}

class Drone
{
    public:

    Drone();
    ~Drone();

    uint64_t iterate();
    
    Iter getstate();
    void setstate(Iter state);
    Param getparams();
    void setparams(Param config);

    private:

    FileBuffer dat;
    FileBuffer err;

    Iter curr;
    Iter prev;
    Param prm;

    PID zpid;
    PID hpid;
    PID ppid;
    PID rpid;

    RateLimiter mrate[4];
};

#endif // DRONE_H
