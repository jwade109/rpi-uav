#include <iostream>
#include <iomanip>
#include <algorithm>
#include <uav/logging>

bool not_event(const uav::archive& a)
{
    return !(a.name() == "Error" ||
             a.name() == "Debug" ||
             a.name() == "Info");
}

int main()
{
    auto archives = uav::restore();
    auto pend = std::remove_if(begin(archives), end(archives), not_event);
    std::string str;
    std::cout << std::left;
    for (auto p = begin(archives); p != pend; ++p)
    {
        *p >> str;
        std::cout << std::setw(15) << p->name() << str << std::endl;
    }
    return 0;
}
