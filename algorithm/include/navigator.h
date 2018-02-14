#ifndef NAVIGATOR_H
#define NAVIGATOR_H

#include <iostream>
#include <array>

#include <Eigen/Core>

#include <uav/algorithm>
#include <uav/hardware>

namespace uav
{

class navigator
{
    public:

    navigator() : navigator(50) { }

    navigator(uint8_t freq) :
        _freq(freq),
        _first(true),
        _dynamic(false),
        _last_mark(0),
        easting(freq, 10, 0.1, 10), northing(freq, 10, 0.1, 10),
        ge(0), gn(0),
        east_sum(freq), north_sum(freq),
        gps_baro(freq),
        vup(freq), vup_lpf(0.25),
        theta{uav::range_accumulator(360),
              uav::range_accumulator(360),
              uav::range_accumulator(360)},
        omega{uav::derivative<1>(freq),
              uav::derivative<1>(freq),
              uav::derivative<1>(freq)},
        omega_lpf{uav::low_pass(0.05),
                  uav::low_pass(0.05),
                  uav::low_pass(0.05)},
        _drift(0, 0, 0) {}

    void predict(std::array<double, 4> thrust, double radius)
    {
        // takes thrust in newtons, radius in meters
        auto Z = Eigen::Vector3d::UnitZ();
        auto Fm1 = thrust[0] * Z, Fm2 = thrust[1] * Z,
             Fm3 = thrust[2] * Z, Fm4 = thrust[3] * Z;
        auto Lm1 = radius * Eigen::Vector3d(1, 1, 0).normalized();
        Eigen::Vector3d Lm2(-Lm1(0),  Lm1(1), 0),
                        Lm3(-Lm1(0), -Lm1(1), 0),
                        Lm4( Lm1(0), -Lm1(1), 0);
        
        _body.clear_forces();
        _body.apply_wrench(Fm1, Lm1, frame::body);
        _body.apply_wrench(Fm2, Lm2, frame::body);
        _body.apply_wrench(Fm3, Lm3, frame::body);
        _body.apply_wrench(Fm4, Lm4, frame::body);
        _body.apply_force(-9.81 * _body.mass() * Z, frame::inertial);
        _body.step(period());
        _body.clear_forces();
        if (_body.displacement()(2) < 0)
        {
            _body.displacement()(2) = _body.velocity()(2) = 0;   
        }
        reconcile_displacement();
    }

    void update(const uav::raw_data& data)
    {
        Eigen::Vector3d attitude;
        attitude << data.ard.euler.x(),
            data.ard.euler.y(), data.ard.euler.z();
        _body.quat() = deg2quat(attitude);

        for (int i = 0; i < 3; i++)
        {
            double angle = theta[i].step(attitude(i));
            double old_rate = omega[i].value;
            double rate = omega[i].step(angle);
            if (i == 0)
            {
                double alpha = (rate - old_rate)*_freq;
                if (std::abs(alpha) > 100000) { omega[i].value = old_rate; }
            }
            _body.turn_rate()(i) = omega_lpf[i].step(omega[i].value, 1.0/_freq);
            if (rate == 0)
                _body.turn_rate()(i) = omega_lpf[i].value = 0;
        }

        _dynamic = _body.turn_rate().norm() > 3;

        if (_first)
        {
            _home_point = _position = data.gps.gga.pos;
            _first = false;
        }

        if (!_dynamic)
        {
            _drift = data.gps.gga.pos - _position;
            _body.velocity() << 0, 0, 0;
            return;
        }

        _position = data.gps.gga.pos - _drift;
        _body.displacement() = _position - _home_point;

        auto hdg = uav::angle::degrees(90 - data.gps.rmc.track_angle);
        double mps = data.gps.rmc.ground_speed/2;
        _body.velocity() << mps*std::cos(hdg), mps*std::sin(hdg), 0;

        ge += _body.velocity()(0)/_freq;
        gn += _body.velocity()(1)/_freq;
        if (data.gps.gga.newflag)
        {
            if (std::abs(ge - _body.displacement()(0)) > 0.25)
                ge = _body.displacement()(0);
            if (std::abs(gn - _body.displacement()(1)) > 0.25)
                gn = _body.displacement()(1);
        }
        _body.displacement()(2) = gps_baro.step(_body.displacement()(2),
            uav::altitude(data.bmp.pressure), uav::altitude(data.ard.pres));
        _body.velocity()(2) = vup.step(_body.displacement()(2));
        _body.velocity()(2) = vup_lpf.step(_body.velocity()(2), 1.0/_freq);

        _body.displacement()(0) = east_sum(easting.seek(east_sum.value, ge));
        _body.displacement()(1) = north_sum(northing.seek(north_sum.value, gn));
        _position = _home_point + _body.displacement();
    }

    void declare_home(const uav::coordinate& home)
    {
        _home_point = home;
        _body.displacement() = _home_point - _position; 
    }

    void declare_home()
    {
        declare_home(_position);
    }

    void reconcile_position(const uav::coordinate& pos)
    {
        auto _old = _body.displacement();
        _position = pos;
        _body.displacement() = _position - _home_point;
        _drift += _old - _body.displacement();
    }
    
    void reconcile_position()
    {
        reconcile_position(_position);
    }

    void reconcile_displacement(const Eigen::Vector3d& disp)
    {
        reconcile_position(_home_point + disp);
    }
    
    void reconcile_displacement()
    {
        reconcile_displacement(_body.displacement());
    }

    bool dynamic() const
    {
        return _dynamic;
    }

    std::chrono::milliseconds period() const
    {
        return std::chrono::milliseconds(1000/_freq);
    }

    std::chrono::milliseconds tow() const
    {
        uint64_t unix_ms = std::chrono::duration_cast
            <std::chrono::milliseconds>
            (std::chrono::system_clock::now().time_since_epoch()).count();
        uint64_t gps_ms = unix_ms - 315964800000 + 17900;
        uint64_t gps_tow = gps_ms % 604800000;
        return std::chrono::milliseconds(gps_tow);
    }

    const freebody& body() const { return _body; }
    freebody& body() { return _body; }

    Eigen::Vector3d displacement() const { return _body.displacement(); }
    Eigen::Vector3d& displacement() { return _body.displacement(); }

    uav::coordinate position() const { return _position; }
    uav::coordinate& position() { return _position; }

    Eigen::Vector3d velocity() const { return _body.velocity(); }
    Eigen::Vector3d& velocity() { return _body.velocity(); }

    Eigen::Quaterniond quat() const { return _body.quat(); }
    Eigen::Quaterniond& quat() { return _body.quat(); }

    Eigen::Vector3d attitude() const { return _body.attitude(); }

    Eigen::Vector3d turn_rate() const { return _body.turn_rate(); }
    Eigen::Vector3d& turn_rate() { return _body.turn_rate(); }

    Eigen::Vector3d drift() const { return _drift; }
    Eigen::Vector3d& drift() { return _drift; }

    uav::coordinate home_point() const { return _home_point; }
    uav::coordinate& home_point() { return _home_point; }

    private:

    const uint8_t _freq;
    bool _first, _dynamic;
    uint64_t _last_mark;

    // for smoothing gps measurements
    pid_controller easting, northing;
    double ge, gn;
    uav::integral<2> east_sum, north_sum;
    uav::gps_baro_filter gps_baro;
    uav::derivative<1> vup;
    uav::low_pass vup_lpf;

    // for producing smooth omega x, y, z values
    std::array<uav::range_accumulator, 3> theta;
    std::array<uav::derivative<1>, 3> omega;
    std::array<uav::low_pass, 3> omega_lpf;

    // physics
    uav::freebody _body;
    uav::coordinate _home_point, _position;
    Eigen::Vector3d _drift;
};

} // namespace uav

#endif // NAVIGATOR_H
