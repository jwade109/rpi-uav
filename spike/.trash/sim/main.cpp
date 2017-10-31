#include <iostream>

#include "freebody.h"

int main()
{
    using namespace uav;
    using namespace imu;

    freebody fb;
    std::cout << "t x y z h p r" << std::endl;
    while (fb.time < 1000000 * 15)
    {
        fb.apply(gravity);
        fb.apply(moment{{0, 0, 0.1}, rframe::body});
        std::cout << fb.time << " " << fb.pos << " " << fb.euler << std::endl;
        fb.step(1000);
    }
    std::cerr << fb << std::endl;

    fb.reset();
    std::cerr << fb << std::endl;

    return 0;
}
