#ifndef FREEBODY_H
#define FREEBODY_H

#include <chrono>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <uav/math>

namespace uav
{

enum frame : bool { inertial = false, body = true };

class freebody
{
    public:

    freebody() :
        _mass(1),
        _moment_inertia(1, 1, 1),
        _displacement(0, 0, 0),
        _velocity(0, 0, 0),
        _acceleration(0, 0, 0),
        _quat(1, 0, 0, 0),
        _turn_rate(0, 0, 0),
        _angular_acceleration(0, 0, 0) { }

    freebody(const freebody& other) :
        _mass(other.mass()),
        _moment_inertia(other.moment_inertia()),
        _displacement(other.displacement()),
        _velocity(other.velocity()),
        _acceleration(other.acceleration()),
        _quat(other.quat()),
        _turn_rate(other.turn_rate()),
        _angular_acceleration(other.angular_acceleration()) { }

    freebody& operator = (const freebody& other)
    {
        _mass = other.mass();
        _moment_inertia = other.moment_inertia();
        _displacement = other.displacement();
        _velocity = other.velocity();
        _acceleration = other.acceleration();
        _quat = other.quat();
        _turn_rate = other.turn_rate();
        _angular_acceleration = other.angular_acceleration();
    }

    double mass() const { return _mass; }
    double& mass() { return _mass; }

    Eigen::Vector3d moment_inertia() const { return _moment_inertia; }
    Eigen::Vector3d& moment_inertia() { return _moment_inertia; }

    Eigen::Vector3d displacement() const { return _displacement; }
    Eigen::Vector3d& displacement() { return _displacement; }

    Eigen::Vector3d velocity() const { return _velocity; }
    Eigen::Vector3d& velocity() { return _velocity; }

    Eigen::Vector3d acceleration() const { return _acceleration; }
    Eigen::Vector3d& acceleration() { return _acceleration; }

    Eigen::Vector3d attitude() const { return uav::quat2deg(_quat); }

    Eigen::Vector3d turn_rate() const { return _turn_rate; }
    Eigen::Vector3d& turn_rate() { return _turn_rate; }

    Eigen::Vector3d angular_acceleration() const { return _angular_acceleration; }
    Eigen::Vector3d& angular_acceleration() { return _angular_acceleration; }

    Eigen::Quaterniond quat() const { return _quat; }
    Eigen::Quaterniond& quat() { return _quat; }

    template <typename Rep, typename Period>
    void step(std::chrono::duration<Rep, Period> dur)
    {
        double dt = std::chrono::duration_cast
            <std::chrono::duration<double>>(dur).count();

        double theta = _turn_rate.norm() * dt;
        auto axis = _turn_rate.normalized();
        _quat = (_quat * Eigen::AngleAxisd(theta, axis)).normalized();

        _turn_rate += dt * _angular_acceleration;
        _velocity += dt * _acceleration;
        _displacement += dt * _velocity;
    }

    void apply_force(const Eigen::Vector3d& force, bool frame)
    {
        _acceleration += (frame == inertial ? force : _quat * force) / _mass;
    }
    
    void apply_moment(const Eigen::Vector3d& moment, bool frame)
    {
        _angular_acceleration += (frame == inertial ?
            moment : _quat * moment).cwiseQuotient(_moment_inertia);
    }

    void apply_wrench(const Eigen::Vector3d& force,
        const Eigen::Vector3d& lever, bool frame)
    {
        apply_force(force, frame);
        apply_moment(lever.cross(force), frame);
    }
    
    void clear_forces()
    {
        _acceleration = _angular_acceleration = Eigen::Vector3d::Zero();
    }
    
    private:

    double _mass;
    Eigen::Vector3d _moment_inertia;
    Eigen::Vector3d _displacement, _velocity, _acceleration;
    Eigen::Quaterniond _quat;
    Eigen::Vector3d _turn_rate, _angular_acceleration;
};

} // namespace uav

#endif // FREEBODY_H
