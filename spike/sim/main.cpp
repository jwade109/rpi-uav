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
    struct force_moment
    {
        imu::Vector<3> force, lever;
    };

    class freebody
    {
        public:

        double time;
        double mass;
        imu::Vector<3> I_moment;

        // pos: position in inertial reference frame, in m
        // vel: velocity in inertial reference frame, in m/s
        // omega: angular velocity, in rad/s
        // euler: angular position in euler angles
        imu::Vector<3> pos;
        imu::Vector<3> world_vel, body_vel;
        imu::Vector<3> omega, euler;

        // q: quaternion expressing orientation
        imu::Quaternion q;

        // rotation: transformation from inertial to body frame
        imu::Matrix<3> rotation;

        std::vector<force_moment> forces;

        freebody();
        void step(double dt);
        void apply(const force_moment& fm);
        std::string str();
        std::string strln();
    };
}

uav::freebody::freebody() : time(0), mass(1), I_moment(1, 1, 1)
{
    rotation = q.toMatrix();
}

void uav::freebody::step(double dt)
{
    time += dt;

    imu::Vector<3> net_force, accel, net_moment, alpha;
    for (int i = 0; i < forces.size(); i++)
    {
        // net force is in the body frame
        net_force += forces[i].force;
        net_moment += forces[i].lever.cross(forces[i].force);
    }

    alpha = net_moment / I_moment;
    omega += alpha;

    imu::Quaternion qw;
    qw.fromAngularVelocity(omega, dt);
    qw.normalize();

    q = q * qw;
    q.normalize();

    euler = q.toEuler();
    euler.toDegrees();
    rotation = q.toMatrix();

    // body frame accel and velocity
    accel = net_force / mass;
    body_vel += accel * dt;

    world_vel = rotation.invert() * body_vel;

    pos += world_vel * dt;

    forces.clear();
}

void uav::freebody::apply(const force_moment& fm)
{
    forces.push_back(fm);
}

std::string uav::freebody::str()
{
    std::stringstream ss;
    ss << "time: " << time << " mass: " << mass
       << " I: " << I_moment << std::endl
       << "pos: " << pos << " vel: " << world_vel << std::endl
       << "forces: ";
    for (int i = 0; i < forces.size(); i++)
        ss << "[{" << forces[i].force << "},{"
           << forces[i].lever << "}]";
    return ss.str();
}

std::string uav::freebody::strln()
{
    std::stringstream ss;
    ss << std::fixed;
    ss << time << " | " << pos << " | " << world_vel << "\n\t "
       << euler << " | " << omega;
    return ss.str();
}

int main()
{
    using namespace uav;
    using namespace imu;

    freebody fb;
    const int millis = 2;
    int count = 0;

    // positive heading moment
    // fb.apply({{0, 1, 0}, {1, 0, 0}});
    // fb.apply({{0, -1, 0}, {0, 0, 0}});

    // positive pitch moment
    // fb.apply({{0, 0,  1}, {0, 1, 0}});
    // fb.apply({{0, 0, -1}, {0, 0, 0}});

    // positive roll moment
    fb.apply({{0, 0, -1}, {1, 0, 0}});
    fb.apply({{0, 0,  1}, {0, 0, 0}});

    while (count < 100)
    {
        fb.step(millis/1000.0);
        if (count % 10 == 0) std::cout << fb.strln() << std::endl;
        count++;
    }
    return 0;
}
