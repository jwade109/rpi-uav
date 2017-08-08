#include <stdio.h>
#include <assert.h>
#include <string.h>
#include <timeutil.h>
#include <filebuffer.h>
#include <dtypes.h>

int main()
{
    std::string div = "END_OF_ITER";

    Param prm = {50, 10, 12,
                {1.2, 3.4, 5.6, 7.8}, {2.1, 4.3, 6.5, 8.7},
                {2.2, 4.4, 6.6, 8.8}, {1.1, 3.3, 5.5, 7.7},
                 0.3, 0.65, 300, 37};

    Iter step = {459332040, 112.3, 109.71, 8.221, 43.21, 2.3, 0.95,
                 223, 150, 20, -40.5, 21.4, -30.9, 55.99, -6.03, 14.0,
                {45, 56, 97, 33}};

    char prmbuf[sizeof(Param)];
    int prmlen = tobuffer(prm, prmbuf);

    FileBuffer params("prm.bin", sizeof(Param));
    params.push(prmbuf, prmlen);
    params.flush();
    printf("Wrote parameter file prm.bin\n");
    
    FileBuffer logb("log.bin", 1000);
    logb.begin();
    FileBuffer logt("log.txt", 1000);
    logt.begin();

    char logbuf[sizeof(Iter)];
    int itlen = tobuffer(step, logbuf);

    uint64_t start = unixtime();
    const int freq = 250;
    const int wait = 1000/freq;
    const int maxtime = 60;
    const int maxsteps = maxtime * freq;
    printf("Writing steps for %d secs %d Hz\n", maxtime, freq);
    for (int i = 0; i < maxsteps; i++)
    {
        memset(&step, 0, sizeof(step));
        step.t = start;
        tobuffer(step, logbuf);
        logb.push(logbuf, itlen);
        logb.push(div);
        
        std::string ss = tostring(step);
        logt.push(ss + "\n");

        waituntil(start+=wait);
        printf("\rWriting step (%d/%d)", i, maxsteps);
    }
    logb.flush();

    return 0;
}
