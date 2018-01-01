#ifndef INS_H
#define INS_H

#include <iostream>
#include <string>
#include <array>

#include <Eigen/Core>

#include <uav/algorithm>
#include <uav/hardware>

namespace uav
{

class ins
{
    public:

    ins(uint8_t freq) : _freq(freq), first(true),
        omega{uav::derivative<1>(freq),
        uav::derivative<1>(freq), uav::derivative<1>(freq)},
        omega_lpf{uav::low_pass(0.05), uav::low_pass(0.05), uav::low_pass(0.05)}
    {
        reset();
    }

    void reset()
    {
        _displacement = Eigen::Vector3d::Zero();
        _velocity = Eigen::Vector3d::Zero();
        _attitude = Eigen::Vector3d::Zero();
        _turn_rate = Eigen::Vector3d::Zero();
    }

    bool begin()
    {
        return sensors.begin() == 0;
    }

    void update()
    {
        auto data = sensors.get();
        _position = data.gps.gga.pos;
        if (first) _home_point = _position;
        _displacement = _position - _home_point;
        
        auto hdg = uav::angle::degrees(90 - data.gps.rmc.track_angle);
        double mps = data.gps.rmc.ground_speed/2;
        _velocity(0) = mps*std::cos(hdg);
        _velocity(1) = mps*std::sin(hdg);

        _attitude(0) = data.ard.euler.x();
        _attitude(1) = data.ard.euler.y();
        _attitude(2) = data.ard.euler.z();
        for (int i = 0; i < 3; i++)
        {
            double rate = omega[i].step(_attitude(i));
            _turn_rate(i) = omega_lpf[i].step(rate, 1.0/_freq);
        }
        first = false;
    }

    void declare_home()
    {
        _home_point = sensors.get().gps.gga.pos;
    }

    void declare_home(const uav::coordinate& home)
    {
        _home_point = home;
    }

    std::chrono::milliseconds tow()
    {
        uint64_t unix_ms = std::chrono::duration_cast
            <std::chrono::milliseconds>(std::chrono::system_clock::now()
            .time_since_epoch()).count();
        uint64_t gps_ms = unix_ms - 315964800000 + 20000;
        uint64_t gps_tow = gps_ms % 604800000;
        return std::chrono::milliseconds(gps_tow);
    }

    Eigen::Vector3d displacement() const { return _displacement; }
    Eigen::Vector3d& displacement() { return _displacement; }

    uav::coordinate position() const { return _position; }

    Eigen::Vector3d velocity() const { return _velocity; }
    Eigen::Vector3d& velocity() { return _velocity; }

    Eigen::Vector3d attitude() const { return _attitude; }
    Eigen::Vector3d& attitude() { return _attitude; }

    Eigen::Vector3d turn_rate() const { return _turn_rate; }
    Eigen::Vector3d& turn_rate() { return _turn_rate; }

    private:

    const uint8_t _freq;
    bool first;
    uav::sensor_hub sensors;
    std::array<uav::derivative<1>, 3> omega;
    std::array<uav::low_pass, 3> omega_lpf;

    uav::coordinate _home_point;
    uav::coordinate _position;
    Eigen::Vector3d _displacement, _velocity, _attitude, _turn_rate;
};

} // namespace uav

#endif // INS_H
