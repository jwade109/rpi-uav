#ifndef FREEBODY_H
#define FREEBODY_H

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace uav
{

enum frame : bool { inertial = false, body = true };

class freebody
{
    public:

    freebody();

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
    Eigen::Vector3d omega() const;
    Eigen::Vector3d& omega();
    Eigen::Vector3d alpha() const;
    Eigen::Vector3d& alpha();

    Eigen::Vector3d bv() const;
    void bv(const Eigen::Vector3d& body_vel);
    Eigen::Vector3d ba() const;
    void ba(const Eigen::Vector3d& body_accel);
    Eigen::Vector3d balpha() const;
    void balpha(const Eigen::Vector3d& body_alpha);
    Eigen::Vector3d bomega() const;
    void bomega(const Eigen::Vector3d& body_omega);
    Eigen::Vector3d euler() const;
    void euler(const Eigen::Vector3d& new_euler);

    void step(uint64_t micros);
    void stepfor(uint64_t time, uint64_t micros);

    void apply_force(const Eigen::Vector3d& force, bool frame);
    void apply_moment(const Eigen::Vector3d& moment, bool frame);
    void apply_wrench(const Eigen::Vector3d& force,
                      const Eigen::Vector3d& lever, bool frame);

    private:

    double _m;
    Eigen::Vector3d _I;

    Eigen::Vector3d _s, _v, _a;
    Eigen::Quaterniond _q;
    Eigen::Vector3d _omega, _alpha;
};

} // namespace uav

#endif // FREEBODY_H
