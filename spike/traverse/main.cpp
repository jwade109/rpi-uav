#include <iostream>
#include <utility>
#include <tuple>
#include <freebody.h>

const double d2r = M_PI/180;

// returns tilt in degrees
double tilt(imu::Vector<3> Zpp)
{
    return acos(Zpp.dot({0, 0, 1})) * 180/M_PI;
}

// returns tilt in degrees
double tilt(double p, double r)
{
    imu::Matrix<3> m(uav::euler2matrix(imu::Vector<3>(0, p*d2r, r*d2r)));
    imu::Vector<3> Zpp = m.col_to_vector(2);
    return tilt(Zpp);
}

std::pair<imu::Vector<3>, imu::Vector<3>>
traverse(double dx, double dy, double heading, double tilt95, double maxtilt)
{
    auto f = [=](double d)
    {
        return 2 * maxtilt / (1 + exp((log(2/1.95 - 1) / tilt95 * d))) - maxtilt;
    };

    const imu::Vector<3> X(1, 0, 0), Y(0, 1, 0), Z(0, 0, 1);
    imu::Vector<3> S(dx, dy, 0), T(Z.cross(S)),
        Xp(X.rotate(Z, heading * M_PI/180)), Yp(Z.cross(Xp));

    double tilt = f(S.magnitude());

    imu::Vector<3> Xpp(Xp.rotate(T, tilt * M_PI/180)),
        Zpp(Z.rotate(T, tilt * M_PI/180)),
        Ypp(Zpp.cross(Xpp)),
        Zp(Xp.cross(Ypp));

    double pitch = asin(Ypp.z()) * 180/M_PI;
    double roll = atan2(Xp.dot(Zpp), Zp.dot(Zpp)) * 180/M_PI;
    return {{heading, pitch, roll}, Zpp};
}

int main()
{
    imu::Vector<3> euler, Zpp;
    double dx, dy, hdg, tilt95, max;
    std::cout << "dx: " << std::flush;
    std::cin >> dx;
    std::cout << "dy: " << std::flush;
    std::cin >> dy;
    std::cout << "hdg: " << std::flush;
    std::cin >> hdg;
    std::cout << "tilt95: " << std::flush;
    std::cin >> tilt95;
    std::cout << "max: " << std::flush;
    std::cin >> max;
    std::tie(euler, Zpp) = traverse(dx, dy, hdg, tilt95, max);
    std::cout << euler << std::endl;
    return 0;
}
