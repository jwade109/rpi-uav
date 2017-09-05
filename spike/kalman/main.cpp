#include <iostream>
#include <string>
#include "kalman.h"

int main()
{
    Eigen::Matrix<double, 2, 1> init = {2, 3};
    std::cout << init << std::endl;
    kalman<3, 2> k(init);
    auto x = {2, 4, -1};
    std::cout << x << std::endl;
    std::cout << k.step(x) << std::endl;
    return 0;
}
