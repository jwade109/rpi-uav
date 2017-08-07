#include <stdio.h>
#include <assert.h>
#include <string.h>
#include <drone.h>

int tobuffer(Param prm, char* buffer)
{
    int wptr = 0;
    memcpy(buffer, &prm.freq, sizeof(prm.freq));
    wptr += sizeof(prm.freq);
    memcpy(buffer + wptr, &prm.z1h, sizeof(prm.z1h));
    wptr += sizeof(prm.z1h);
    memcpy(buffer + wptr, &prm.z2h, sizeof(prm.z2h));
    wptr += sizeof(prm.z2h);
    memcpy(buffer + wptr, prm.zpidg, sizeof(prm.zpidg[0]) * 4);
    wptr += (sizeof(prm.zpidg[0]) * 4);
    memcpy(buffer + wptr, prm.hpidg, sizeof(prm.hpidg[0]) * 4);
    wptr += (sizeof(prm.hpidg[0]) * 4);
    memcpy(buffer + wptr, prm.ppidg, sizeof(prm.ppidg[0]) * 4);
    wptr += (sizeof(prm.ppidg[0]) * 4);
    memcpy(buffer + wptr, prm.rpidg, sizeof(prm.rpidg[0]) * 4);
    wptr += (sizeof(prm.rpidg[0]) * 4);
    memcpy(buffer + wptr, &prm.gz_lpf, sizeof(prm.gz_lpf));
    wptr += sizeof(prm.gz_lpf);
    memcpy(buffer + wptr, &prm.gz_wam, sizeof(prm.gz_wam));
    wptr += sizeof(prm.gz_wam);
    memcpy(buffer + wptr, &prm.maxmrate, sizeof(prm.maxmrate));
    wptr += sizeof(prm.maxmrate);
    memcpy(buffer + wptr, &prm.mg, sizeof(prm.mg));
    wptr += sizeof(prm.mg);
    return wptr;
}

int frombuffer(Param& prm, char* buffer)
{
    int rptr = 0;
    memcpy(&prm.freq, buffer, sizeof(prm.freq));
    rptr += sizeof(prm.freq);
    memcpy(&prm.z1h, buffer + rptr, sizeof(prm.z1h));
    rptr += sizeof(prm.z1h);
    memcpy(&prm.z2h, buffer + rptr, sizeof(prm.z2h));
    rptr += sizeof(prm.z2h);
    memcpy(prm.zpidg, buffer + rptr, sizeof(prm.zpidg[0]) * 4);
    rptr += (sizeof(prm.zpidg[0]) * 4);
    memcpy(prm.hpidg, buffer + rptr, sizeof(prm.hpidg[0]) * 4);
    rptr += (sizeof(prm.hpidg[0]) * 4);
    memcpy(prm.ppidg, buffer + rptr, sizeof(prm.ppidg[0]) * 4);
    rptr += (sizeof(prm.ppidg[0]) * 4);
    memcpy(prm.rpidg, buffer + rptr, sizeof(prm.rpidg[0]) * 4);
    rptr += (sizeof(prm.rpidg[0]) * 4);
    memcpy(&prm.gz_lpf, buffer + rptr, sizeof(prm.gz_lpf));
    rptr += sizeof(prm.gz_lpf);
    memcpy(&prm.gz_wam, buffer + rptr, sizeof(prm.gz_wam));
    rptr += sizeof(prm.gz_wam);
    memcpy(&prm.maxmrate, buffer + rptr, sizeof(prm.maxmrate));
    rptr += sizeof(prm.maxmrate);
    memcpy(&prm.mg, buffer + rptr, sizeof(prm.mg));
    rptr += sizeof(prm.mg);
    return rptr;
}

int main()
{
    FileBuffer bin("prm.bin");

    Param prm = {50, 10, 12,
                {1.2, 3.4, 5.6, 7.8},
                {2.1, 4.3, 6.5, 8.7},
                {2.2, 4.4, 6.6, 8.8},
                {1.1, 3.3, 5.5, 7.7},
                0.3, 0.65, 300, 37};

    char buffer[sizeof(Param)];
    
    int ret1 = tobuffer(prm, buffer);
    Param newprm;
    int ret2 = frombuffer(newprm, buffer);

    FileBuffer file("prm.bin");
    file.push(buffer, ret1);
    file.flush();

    assert(ret1 == ret2);
    assert(prm.freq == newprm.freq);

    return 0;
}
