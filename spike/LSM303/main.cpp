#include <iostream>
#include <errno.h>
#include <lsm.h>

int main()
{
    uav::lsm303 lsm;
    if (!lsm.begin())
        std::cerr << "Failed to start LSM303!" << std::endl;
    return 0;
}
