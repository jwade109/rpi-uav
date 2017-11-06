#ifndef FREEBODY_H
#define FREEBODY_H

#include <Eigen/Core>
#include <Eigen/Geometry>

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

        freebody();


        // intrinsic properties
        double m() const;
        double& m();
        Eigen::Vector3d I() const;
        Eigen::Vector3d& I();
        Eigen::Vector3d s() const;
        Eigen::Vector3d& s();
        Eigen::Vector3d v() const;
        Eigen::Vector3d& v();
        Eigen::Vector3d a() const;
        Eigen::Vector3d& a();
        Eigen::Quaterniond q() const;
        Eigen::Quaterniond& q();
        Eigen::Vector3d a() const;
        Eigen::Vector3d a() const;
        Eigen::Vector3d& a();
        Eigen::Vector3d& a();

        private:

        double _m;
        Eigen::Vector3d _I;

        Eigen::Vector3d _s, _v, _a;
        Eigen::Quaterniond _q;
        Eigen::Vector3d _omega, _alpha;
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

#endif // FREEBODY_H
