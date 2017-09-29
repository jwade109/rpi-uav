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
        imu::Vector<3> pos, vel, accel;
        imu::Quaternion q;
        imu::Vector<3> omega, alpha;

        imu::Vector<3> X, Y, Z;
        double heading, pitch, roll;

        // rotation: transformation from inertial to body frame
        imu::Matrix<3> rotation;

        std::vector<force_moment> forces;

        freebody();
        void step(uint64_t micros);
        void apply(const force_moment& fm);

        void setbodyvel(const imu::Vector<3>& bvel);
        void setbodyomega(const imu::Vector<3>& bomega);

        std::string str() const;
        std::string strln() const;
    };
}

uav::freebody::freebody() : time(0), mass(1), I_moment(1, 1, 1),
    heading(0), pitch(0), roll(0)
{
    rotation = q.toMatrix();
}

void uav::freebody::step(uint64_t micros)
{
    time += micros;

    imu::Vector<3> net_force, net_moment;
    for (int i = 0; i < forces.size(); i++)
    {
        // net force is in the inertial frame
        if (forces[i].frame == rframe::inertial)
        {
            net_force += forces[i].force;
            net_moment += forces[i].lever.cross(forces[i].force);

            // std::cout << "inertial: " << forces[i].force << std::endl;
        }
        else // forces[i].frame == body
        {
            auto body2world = rotation.invert();
            auto inertial_force = body2world * forces[i].force;
            auto inertial_lever = body2world * forces[i].lever;

            // std::cout << "body: " << inertial_force << std::endl;

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

    X = rotation.col_to_vector(0);
    Y = rotation.col_to_vector(1);
    Z = rotation.col_to_vector(2);

    heading = atan2(rotation(0,1), rotation(1,1)) * 180.0 / M_PI;
    pitch = asin(rotation(2,1)) * 180.0 / M_PI;
    // how to calculate roll angle?

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
       << " pitch: " << pitch
       << " roll: " << roll
       << " hdg: " << heading << std::endl
       << "omega: " << omega << std::endl
       << "quat: " << q;
    return ss.str();
}

std::string uav::freebody::strln() const
{
    std::stringstream ss;
    ss << std::fixed;
    ss << time << " | " << pos << " | " << vel << "\n\t "
       << q << " | " << omega;
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
    const int micros = 50;
    int count = 0;

    // positive heading moment
    // fb.apply({{0,  1, 0}, {1, 0, 0}, rframe::body});
    // fb.apply({{0, -1, 0}, {0, 0, 0}, rframe::body});

    // positive pitch moment
    // fb.apply({{0, 0,  1}, {0, 1, 0}});
    // fb.apply({{0, 0, -1}, {0, 0, 0}});

    // positive roll moment
    // fb.apply({{0, 0, -1}, {1, 0, 0}});
    // fb.apply({{0, 0,  1}, {0, 0, 0}});

    fb.omega = {0, 0,1};
    fb.step(M_PI * 1000000 / 4);
    fb.setbodyomega({1, 0, 0});
    fb.step(M_PI * 1000000 / 6);
    std::cout << fb << std::endl;
    fb.setbodyomega({0, 1, 0});
    fb.step(M_PI * 1000000 / 12);
    std::cout << fb << std::endl;
    return 0;

    while (true)
    {
        auto mg = gravity;
        mg.force *= fb.mass;
        fb.apply(mg);

        force_moment f = {{0, 1, 0}, {0, 0, 0}, rframe::body};
        fb.apply(f);
        if (fb.time % (1000000/20) == 0)
        {
            // std::cout << f.force << std::endl;
            // std::cout << fb.rotation << std::endl;
            std::cout << fb.q << std::endl;
            std::cout << std::left << std::setw(10) << fb.time/1000000.0
                      << std::left << std::setw(30) << fb.pos
                      << std::left << std::setw(30) << fb.vel
                      << fb.accel << std::endl;
            std::this_thread::sleep_for(std::chrono::milliseconds(1000/20));
        }
        fb.step(micros);
    }
    return 0;
}
