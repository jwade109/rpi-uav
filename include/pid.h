#ifndef PID_H
#define PID_H

#include <stdint.h>
#include <limits>
#include <vector.h>

class pid_controller
{
    public:

    double Kp, Ki, Kd;
    double prev_error;
    double integral;
    bool has_prev;
    uint16_t max_int;
    double p_response, i_response, d_response;

    pid_controller(double Kp, double Ki, double Kd, uint16_t max = -1);
    void reset();
    double seek(double actual, double setpoint, double dt);
};

template <uint8_t N> class pid_vector
{
    public:

    double Kp, Ki, Kd;
    imu::Vector<N> prev_error, integral;
    bool has_prev;
    imu::Vector<N> max_int;
    imu::Vector<N> p_response, i_response, d_response;

    pid_vector(double Kp, double Ki, double Kd);
    pid_vector(double Kp, double Ki, double Kd, imu::Vector<N> max);
    void reset();
    static uint8_t order();
    imu::Vector<N> seek(imu::Vector<N> actual,
                        imu::Vector<N> setpoint, double dt);
};

template <uint8_t N>
pid_vector<N>::pid_vector(double Kp, double Ki, double Kd) :
    Kp(Kp), Ki(Ki), Kd(Kd)
{
    double max = std::numeric_limits<double>::max();
    max_int = {max, max, max};
    reset();
}

template <uint8_t N>
pid_vector<N>::pid_vector(double Kp, double Ki, double Kd,
                          imu::Vector<N> max) :
    Kp(Kp), Ki(Ki), Kd(Kd), max_int(max)
{
    reset();
}

template <uint8_t N>
void pid_vector<N>::reset()
{
    prev_error = imu::Vector<N>();
    integral = imu::Vector<N>();
    has_prev = false;
    p_response = i_response = d_response = imu::Vector<N>();
}

template <uint8_t N>
uint8_t pid_vector<N>::order()
{
    return N;
}

template <uint8_t N>
imu::Vector<N> pid_vector<N>::seek(imu::Vector<N> actual,
                           imu::Vector<N> setpoint, double dt)
{
    auto error = setpoint - actual, error_rate = imu::Vector<N>();

    // assign steady state error with windup guards
    integral += error * dt;

    auto constrain = [](double val, double min, double max)
    {
        return val > max ? max : val < min ? min : val;
    };

    for (uint8_t i = 0; i < N; i++)
        integral(i) = constrain(integral(i), -max_int(i), max_int(i));

    // calculate error_rate if possible
    if (has_prev)
        error_rate = (error - prev_error) / dt;

    // store error for calculating error_rate next iteration
    prev_error = error;
    has_prev = true;

    p_response = error * Kp;
    i_response = integral * Ki;
    d_response = error_rate * Kd;

    return p_response + i_response + d_response;
}

#endif
