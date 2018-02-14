#include <iostream>

#include <freebody.h>
#include <navigator.h>

using namespace uav;
using namespace imu;

void print_quat(std::ostream& os, const Eigen::Quaterniond& q)
{
    os << q.w() << " " << q.x() << " " << q.y() << " " << q.z() << " ";
}

void print_vector(std::ostream& os, const Eigen::Vector3d& v)
{
    for (int i = 0; i < 3; ++i)
    {
        os << v(i) << " ";
    }
}
void print_nav(std::ostream& os, const uav::navigator& nav)
{
    os << nav.tow().count() << " " << nav.position() << " ";
    print_vector(os, nav.velocity());
    print_vector(os, nav.attitude());
    print_vector(os, nav.turn_rate());
}

int main()
{
    uav::navigator nav(50);
    std::cout << std::fixed << std::setprecision(2);
    nav.declare_home({39.149075, -77.619147, 150.0});
    nav.reconcile_position({39.149068, -77.619151, 147.3});

    uint64_t last = nav.tow().count();

    while (1)
    {
        std::cout << nav.steady_time().time_since_epoch().count();
        if (nav.mark())
        {
            std::cout << " " << nav.tow().count();
        }
        std::cout << std::endl;
    }

    return 0;
}
