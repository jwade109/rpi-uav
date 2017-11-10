#include <iostream>

#include "optional.h"

int main()
{
    uav::optional<const char*> a("WOW!", false);

    a = "WHOA";
    a.is() = true;

    if (a.is())
    std::cout << a << std::endl;
    else std::cout << "..." << std::endl;

    auto z = uav::make_optional("WOW");
    std::cout << z << std::endl;

    uav::optional<double> b = 50 - 23.4;
    uav::optional<int> c = 6;
    std::cout << b << std::endl;
    std::cout << c << std::endl;
    std::cout << b - c << std::endl;

    return 0;
}
