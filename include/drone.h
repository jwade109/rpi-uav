#ifndef DRONE_H
#define DRONE_H

#include <stdint.h>
#include <pid.h>
#include <filebuffer.h>
#include <filters.h>
#include <ardimu.h>
#include <bmp.h>
#include <dtypes.h>

class Drone
{
    public:

    Drone(Iter initial, Param cfg);
    ~Drone();

    int align();
    int iterate();
    
    Iter getstate();
    void setstate(Iter state);
    Param getparams();

    private:

    Arduino imu;
    BMP085 bmp;
    Iter curr, prev;
    Param prm;
    FileBuffer dat, err;
    PID zpid, hpid, ppid, rpid;
    LowPassFilter zlpf;
    RateLimiter mr1, mr2, mr3, mr4;

    void gettargets(Iter& state);
    void writeparams(Param cfg);
};

#endif // DRONE_H
