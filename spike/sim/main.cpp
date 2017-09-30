#include <iostream>
#include <iomanip>
#include <vector>
#include <string>
#include <sstream>
#include <chrono>
#include <thread>

#include "utility/imumaths.h"

namespace uav
{
    enum class rframe
    {
        inertial, body
    };

    struct force_moment
    {
        imu::Vector<3> force, lever;
        rframe frame;
    };

    const force_moment M0 = {{0, 0, 1}, { 1,  1, 0}, rframe::body};
    const force_moment M1 = {{0, 0, 1}, {-1,  1, 0}, rframe::body};
    const force_moment M2 = {{0, 0, 1}, {-1, -1, 0}, rframe::body};
    const force_moment M3 = {{0, 0, 1}, { 1, -1, 0}, rframe::body};

    const force_moment gravity = {{0, 0, -9.81}, {0, 0, 0}, rframe::inertial};

    class freebody
    {
        public:

        uint64_t time;
        double mass;
        imu::Vector<3> I_moment;

        // pos: position in inertial reference frame, in m
        // vel: velocity in inertial reference frame, in m/s
        // omega: angular velocity, in rad/s
        // euler: angular position in euler angles
        // ...with rotation order Z-X'-Y''
        imu::Vector<3> pos, vel, accel;
        imu::Quaternion q;
        imu::Vector<3> euler, omega, alpha;
        // euler: z is heading, x is pitch, y is roll

        imu::Vector<3> X, Y, Z;

        // rotation: transformation from inertial to body frame
        imu::Matrix<3> rotation;

        std::vector<force_moment> forces;

        freebody();
        void step(uint64_t micros);
        void apply(const force_moment& fm);

        void setbodyvel(const imu::Vector<3>& bvel);
        void setbodyomega(const imu::Vector<3>& bomega);

        std::string str() const;
    };
}

uav::freebody::freebody() : time(0), mass(1), I_moment(1, 1, 1)
{
    rotation = q.toMatrix();
}

void uav::freebody::step(uint64_t micros)
{
    time += micros;

    imu::Vector<3> net_force, net_moment;
    for (size_t i = 0; i < forces.size(); i++)
    {
        // net force is in the inertial frame
        if (forces[i].frame == rframe::inertial)
        {
            net_force += forces[i].force;
            net_moment += forces[i].lever.cross(forces[i].force);
        }
        else // forces[i].frame == body
        {
            auto inertial_force = rotation * forces[i].force;
            auto inertial_lever = rotation * forces[i].lever;

            net_force += inertial_force;
            net_moment += inertial_lever.cross(inertial_force);
        }
    }

    alpha = net_moment / I_moment;
    omega += alpha;

    double dt = micros/1000000.0;
    imu::Quaternion qw;
    qw.fromAngularVelocity(omega, dt);
    qw.normalize();

    // world frame accel and velocity
    accel = net_force / mass;
    vel += accel * dt;
    pos += vel * dt;

    q = qw * q;
    q.normalize();
    rotation = q.toMatrix();

    // rotated body frame basis
    imu::Vector<3> Xpp(rotation.col_to_vector(0)),
        Ypp(rotation.col_to_vector(1)),
        Zpp(rotation.col_to_vector(2));

    imu::Vector<3> Yp(Ypp.x(), Ypp.y(), 0);
    Yp.normalize();
    imu::Vector<3> Xp(Yp.cross({0, 0, 1})),
        Zp(Xp.cross(Ypp));

    euler.x() = atan2(Xp.y(), Xp.x()) * 180.0 / M_PI;
    if (euler.x() < 0) euler.x() += 360;
    euler.y() = asin(Ypp.z()) * 180.0 / M_PI;
    euler.z() = atan2(Xp.dot(Zpp), Zp.dot(Zpp)) * 180.0 / M_PI;

    X = Xpp;
    Y = Ypp;
    Z = Zpp;

    forces.clear();
}

void uav::freebody::apply(const force_moment& fm)
{
    forces.push_back(fm);
}

void uav::freebody::setbodyvel(const imu::Vector<3>& bvel)
{
    vel = rotation * bvel;
}

void uav::freebody::setbodyomega(const imu::Vector<3>& bomega)
{
    omega = rotation * bomega;
}

std::string uav::freebody::str() const
{
    std::stringstream ss;
    ss << "time: " << time/1000000.0 << " mass: " << mass
       << " I: " << I_moment << std::endl
       << "pos: " << pos << " vel: " << vel << std::endl
       << "Z-X-Y: " << euler << std::endl
       << "omega: " << omega << std::endl
       << "quat: " << q
       << "\nrot:\n" << rotation;
    return ss.str();
}

std::ostream& operator << (std::ostream& os, uav::freebody& fb)
{
    os << fb.str();
    return os;
}

int main()
{
    using namespace uav;
    using namespace imu;

    freebody fb;

    fb.omega = {0, 0,1};
    fb.step(M_PI * 500000 * 30 / 90);
    fb.setbodyomega({1, 0, 0});
    fb.step(M_PI * 500000 * 60 / 90);
    fb.setbodyomega({0, 1, 0});
    std::cout << fb << std::endl;
    std::cout << std::fixed;
    fb.time = 0;

    for (int i = 0; i < 9; i++)
    {
        fb.setbodyomega({1, 0, 0});
        fb.step(M_PI * 500000 * 40 / 90);
        if (i == 8) fb.step(6);
        std::cout << fb.time/1000000.0 << " : "
            << fb.euler << std::endl;
        std::cout << fb.q << std::endl;
    }

    fb.setbodyomega({0, 1, 0});
    fb.step(M_PI * 500000 * 17.4 / 90);
    std::cout << fb << std::endl;

    return 0;
}
