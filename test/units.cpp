#include <iostream>

#include <ardimu.h>
#include <bmp.h>
#include <gps.h>
#include <lsm.h>

int main()
{
    std::cout << "Testing arduino..." << std::endl;
    uav::arduino ard;
    int ret = ard.begin();
    if (ret > 0)
        std::cout << "Test failed, returned: " << ret << std::endl;
    else std::cout << "Test success." << std::endl;

    std::cout << "Testing GPS..." << std::endl;
    uav::gps gps;
    ret = gps.begin();
    if (ret > 0)
        std::cout << "Test failed, returned: " << ret << std::endl;
    else std::cout << "Test success." << std::endl;

    std::cout << "Testing BMP085..." << std::endl;
    uav::bmp085 bmp;
    ret = bmp.begin();
    if (ret > 0)
        std::cout << "Test failed, returned: " << ret << std::endl;
    else std::cout << "Test success." << std::endl;

    std::cout << "Testing LSM303..." << std::endl;
    uav::lsm303 lsm;
    ret = lsm.begin();
    if (ret > 0)
        std::cout << "Test failed, returned: " << ret << std::endl;
    else std::cout << "Test success." << std::endl;
    
    return 0;
}
