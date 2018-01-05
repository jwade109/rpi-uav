#include <iostream>

#include <freebody.h>

using namespace uav;
using namespace imu;

int main()
{
    dronebody db;
    db.step(0);
    db.set(1, 1, 0.9, 1.1);
    db.stepfor(1000000, 1000);
    std::cerr << db << std::endl;

    return 0;
}
