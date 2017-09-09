#include <iostream>
#include <chrono>
#include <thread>

#include "gps.h"

int main()
{
    gps r;
    int ret = r.begin();
    if (ret)
    {
        std::cerr << "Error: " << ret << std::endl;
        return 1;
    }
    while(1);
}
