#include <iostream>
#include <iomanip>
#include <vector>
#include <string>
#include <sstream>
#include <chrono>
#include <thread>

#include "quaternion.h"
#include "vector.h"
#include "matrix.h"

namespace uav
{
    enum class rframe
    {
        inertial, body
    };

    struct force
    {
        imu::Vector<3> force;
        rframe frame;
    };

    struct moment
    {
        imu::Vector<3> moment;
        rframe frame;
    };

    const force gravity = {{0, 0, -9.81}, rframe::inertial};

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
        imu::Quaternion quat;
        imu::Vector<3> euler, omega, alpha;
        // euler: z is heading, x is pitch, y is roll

        imu::Vector<3> X, Y, Z;

        // rotation: transformation from inertial to body frame
        imu::Matrix<3> rotation;

        std::vector<force> forces;
        std::vector<moment> moments;

        freebody();
        
        void step(uint64_t micros);
        void stepfor(uint64_t micros, uint64_t dt);
        void reset();

        void apply(const force& f);
        void apply(const moment& m);
        void apply(const force& f, const imu::Vector<3>& lever);

        imu::Vector<3> bodyvel();
        imu::Vector<3> bodyomega();
        void setbodyvel(const imu::Vector<3>& bvel);
        void setbodyomega(const imu::Vector<3>& bomega);

        std::string str() const;
    };

    struct motor
    {
        enum rdir : bool { CW = true, CCW = false };

        double Izz;
        double omega;
        bool dir;
        imu::Vector<3> lever;
    };

    class dronebody : public freebody
    {
        public:

        motor M0, M1, M2, M3;

        dronebody();
        void step(uint64_t micros);
        void stepfor(uint64_t micros, uint64_t dt);
        void reset();

        double tilt();

        void set(uint8_t n, double omega);
        void set(double o1, double o2, double o3, double o4);
        void set(double omega[4]);

        std::string str() const;
    };

    imu::Matrix<3> euler2matrix(const imu::Vector<3>& euler);

    imu::Vector<3> matrix2euler(const imu::Matrix<3>& m);
}

std::ostream& operator << (std::ostream& os, const uav::motor& m);

std::ostream& operator << (std::ostream& os, const uav::freebody& fb);

std::ostream& operator << (std::ostream& os, const uav::dronebody& fb);

std::ostream& operator << (std::ostream& os, const imu::Quaternion& q);
