#include "angles.h"

int main()
{
    using namespace uav::angle_literals;
    
    auto a = 100_deg;
    std::cout << a.rad() << std::endl;
    return 0;
}
