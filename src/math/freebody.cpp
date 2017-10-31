#include <iomanip>
#include <string>
#include <sstream>

#include "freebody.h"

uav::freebody::freebody() : time(0), mass(1), I_moment(1, 1, 1)
{
    rotation = quat.toMatrix();

    euler = matrix2euler(rotation);
    euler.toDegrees();

    X = rotation.col_to_vector(0);
    Y = rotation.col_to_vector(0);
    Z = rotation.col_to_vector(0);
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

    accel = net_force / mass;
    vel += accel * dt;
    pos += vel * dt;

    quat = qw * quat;
    quat.normalize();
    rotation = quat.toMatrix();

    rotation = quat.toMatrix();

    euler = matrix2euler(rotation);
    euler.toDegrees();

    X = rotation.col_to_vector(0);
    Y = rotation.col_to_vector(0);
    Z = rotation.col_to_vector(0);

    forces.clear();
    moments.clear();
}

void uav::freebody::stepfor(uint64_t micros, uint64_t dt)
{
    uint64_t end = time + micros;
    while (time < end) step(dt);
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
}

void uav::freebody::setbodyvel(const imu::Vector<3>& bvel)
{
    vel = rotation * bvel;
}

void uav::freebody::setbodyomega(const imu::Vector<3>& bomega)
{
    omega = rotation * bomega;
}

imu::Vector<3> uav::freebody::bodyvel()
{
    return rotation.invert() * vel;
}

imu::Vector<3> uav::freebody::bodyomega()
{
    return rotation.invert() * omega;
}

std::string uav::freebody::str() const
{
    std::stringstream ss;
    ss << "time: " << time/1000000.0 << " mass: " << mass
       << " I:    " << I_moment << std::endl
       << "s:     " << pos << std::endl
       << "v:     " << vel << std::endl
       << "a:     " << accel << std::endl
       << "α-β-γ: " << euler << std::endl
       << "ω:     " << omega << std::endl
       << "α:     " << alpha << std::endl
       << "quat:  " << quat;
    return ss.str();
}

uav::dronebody::dronebody() :
    M0{1, 0, motor::CW, { 1,  1, 0}}, M1{1, 0, motor::CCW, {-1,  1, 0}},
    M2{1, 0, motor::CW, {-1, -1, 0}}, M3{1, 0, motor::CCW, { 1, -1, 0}} { }

void uav::dronebody::step(uint64_t micros)
{
    apply(force{gravity.force * mass, rframe::inertial});
    apply(force{{0, 0, M0.omega}, rframe::body}, M0.lever);
    apply(force{{0, 0, M1.omega}, rframe::body}, M1.lever);
    apply(force{{0, 0, M2.omega}, rframe::body}, M2.lever);
    apply(force{{0, 0, M3.omega}, rframe::body}, M3.lever);

    // body angular momentum changes by momentum of motors
    double L_m = 0;
    motor motors[] = {M0, M1, M2, M3};

    for (auto m : motors)
    {
        if (m.dir == motor::CW)
            L_m += m.Izz * m.omega;
        else // CCW
            L_m -= m.Izz * m.omega;
    }

    auto bomega = bodyomega();
    bomega.z() = -L_m/I_moment.z();
    setbodyomega(bomega);

    freebody::step(micros);
}

void uav::dronebody::stepfor(uint64_t micros, uint64_t dt)
{
    uint64_t end = time + micros;
    while (time < end) step(dt);
}

void uav::dronebody::reset()
{
    *this = dronebody();
}

double uav::dronebody::tilt()
{
    return acos(Z.dot({0, 0, 1})) * 180/M_PI;
}

void uav::dronebody::set(uint8_t n, double omega)
{
    switch (n)
    {
        case 0: M0.omega = omega; break;
        case 1: M1.omega = omega; break;
        case 2: M2.omega = omega; break;
        case 3: M3.omega = omega; break;
        default: break;
    }
}

void uav::dronebody::set(double o1, double o2, double o3, double o4)
{
    M0.omega = o1;
    M1.omega = o2;
    M2.omega = o3;
    M3.omega = o4;
}

void uav::dronebody::set(double omega[4])
{
    M0.omega = omega[0];
    M1.omega = omega[1];
    M2.omega = omega[2];
    M3.omega = omega[3];
}

std::string uav::dronebody::str() const
{
    std::stringstream ss;
    ss << freebody::str();
    ss << std::endl << M0 << std::endl << M1 << std::endl
       << M2 << std::endl << M3;
    return ss.str();
}

// euler angles must be in radians, rotation order Z-X'-Y''
imu::Matrix<3> uav::euler2matrix(const imu::Vector<3>& euler)
{
    const imu::Vector<3> X(1, 0, 0), Y(0, 1, 0), Z(0, 0, 1);

    imu::Vector<3> Xp(X.rotate(Z, euler.x())),
        Yp(Y.rotate(Z, euler.x())),
        Ypp(Yp.rotate(Xp, euler.y())),
        Zp(Z.rotate(Xp, euler.y())),
        Xpp(Xp.rotate(Ypp, euler.z())),
        Zpp(Zp.rotate(Ypp, euler.z()));

    imu::Matrix<3> m;
    m.vector_to_col(Xpp, 0);
    m.vector_to_col(Ypp, 1);
    m.vector_to_col(Zpp, 2);
    return m;
}

// returned euler angles are Z-X'-Y'' rotations in radians
imu::Vector<3> uav::matrix2euler(const imu::Matrix<3>& m)
{
    imu::Vector<3> Xpp(m.col_to_vector(0)),
        Ypp(m.col_to_vector(1)),
        Zpp(m.col_to_vector(2)),
        Yp(Ypp.x(), Ypp.y(), 0);
    Yp.normalize();
    imu::Vector<3> Xp(Yp.cross({0, 0, 1})),
        Zp(Xp.cross(Ypp));

    imu::Vector<3> euler;
    euler.x() = atan2(Xp.y(), Xp.x());
    euler.y() = asin(Ypp.z());
    euler.z() = atan2(Xp.dot(Zpp), Zp.dot(Zpp));
    return euler;
}

std::ostream& operator << (std::ostream& os, const uav::motor& m)
{
    os << "[ omega: " << m.omega << " Izz: " << m.Izz << " "
       << (m.dir == uav::motor::CW ? "CW " : "CCW ")
       << m.lever << "]";
    return os;
}

std::ostream& operator << (std::ostream& os, const uav::freebody& fb)
{
    os << fb.str();
    return os;
}

std::ostream& operator << (std::ostream& os, const uav::dronebody& db)
{
    os << db.str();
    return os;
}

std::ostream& operator << (std::ostream& os, const imu::Quaternion& q)
{
    os << "{" << q.w() << ", " << q.x() << ", " << q.y()
       << ", " << q.z() << "}";
    return os;
}

