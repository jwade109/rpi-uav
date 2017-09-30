#include <iomanip>
#include <string>
#include <sstream>

#include <freebody.h>

uav::freebody::freebody() : time(0), mass(1), I_moment(1, 1, 1)
{
    rotation = quat.toMatrix();

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
}

void uav::freebody::step(uint64_t micros)
{
    time += micros;

    imu::Vector<3> net_force, net_moment;
    for (size_t i = 0; i < forces.size(); i++)
    {
        // force is in the inertial frame
        if (forces[i].frame == rframe::inertial)
            net_force += forces[i].force;
        else // forces[i].frame == body
            net_force += rotation * forces[i].force;
    }
    for (size_t i = 0; i < moments.size(); i++)
    {
        // moment is in the inertial frame
        if (moments[i].frame == rframe::inertial)
            net_moment += moments[i].moment;
        else // moments[i].frame == body
            net_moment += rotation * moments[i].moment;
    }

    double dt = micros/1000000.0;

    alpha = net_moment / I_moment;
    omega += alpha * dt;

    imu::Quaternion qw;
    qw.fromAngularVelocity(omega, dt);
    qw.normalize();

    // world frame accel and velocity
    accel = net_force / mass;
    vel += accel * dt;
    pos += vel * dt;

    quat = qw * quat;
    quat.normalize();
    rotation = quat.toMatrix();

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
    moments.clear();
}

void uav::freebody::reset()
{
    *this = freebody();
}

void uav::freebody::apply(const force& f)
{
    forces.push_back(f);
}

void uav::freebody::apply(const moment& m)
{
    moments.push_back(m);
}

// f.frame is the reference frame for both the force AND the lever!
void uav::freebody::apply(const force& f, const imu::Vector<3>& lever)
{
    moment m{lever.cross(f.force), f.frame};
    forces.push_back(f);
    moments.push_back(m);

    // std::cout << "new force: " << f.force << std::endl;
    // std::cout << "new moment: " << m.moment << std::endl;
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
       << "quat: " << quat << std::endl
       << "basis: " << X << " " << Y << " " << Z << std::endl
       << "rot:\n" << rotation;
    return ss.str();
}

std::ostream& operator << (std::ostream& os, const uav::freebody& fb)
{
    os << fb.str();
    return os;
}

std::ostream& operator << (std::ostream& os, const imu::Quaternion& q)
{
    os << "{" << q.w() << ", " << q.x() << ", " << q.y()
       << ", " << q.z() << "}";
    return os;
}

