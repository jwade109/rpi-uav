#include <iostream>
#include <vector>

#include "utility/imumaths.h"

namespace uav
{
    class freebody
    {
        public:

        // pos: position in inertial reference frame, in m
        // vel: velocity in inertial reference frame, in m/s
        // omega: angular velocity, in rad/s
        // euler: angular position in euler angles
        imu::Vector<3> pos, vel, omega, euler;

        // q: quaternion expressing orientation
        imu::Quaternion q;

        // rotation: transformation from inertial to body frame
        imu::Matrix<3> rotation;

        freebody();

        void step(double dt);
    };
}

uav::freebody::freebody()
{
    rotation = q.toMatrix();
}

void uav::freebody::step(double dt)
{
    rotation = q.toMatrix();
    pos += vel * dt;
}

int main()
{
    return 0;
}
