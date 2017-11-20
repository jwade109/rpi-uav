#include <iostream>

#include <uav/hardware>

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

    return 0;
}
