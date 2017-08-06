#include <drone.h>
#include <timeutil.h>
#include <inttypes.h>
#include <cstdio>

int main()
{
    Iter init = {0};

    // example configuration
    Param prm = {100, 0, 0, {0, 0, 0.35, -1}, {0, 0, 1, -1},
                {1, 0, 1, -1}, {1, 0, 1, -1}, 0.3, 0.65, 250, 37};

    Drone d(init, prm);
    if (d.align())
    {
        fprintf(stderr, "Something went wrong\n");
        return 1;
    }

    uint64_t start = unixtime();
    const int wait = 10;

    for (int i = 0; i < 100; i++)
    {
        int ret = d.iterate();
        Iter s = d.getstate();
        printf("%" PRIu64 ": %d %d %d %d\n", s.t,
                (int) s.motors[0], (int) s.motors[1],
                (int) s.motors[2], (int) s.motors[3]);
        waituntil(start+=wait);
    }

    return 0;
}
