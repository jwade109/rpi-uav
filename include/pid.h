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

    pid_controller(double Kp, double Ki, double Kd, double max = NAN);
    void reset();
    double seek(double actual, double setpoint, double dt);
};

#endif
