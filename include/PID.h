#ifndef PID_H
#define PID_H

#include <stdbool.h>
#include <stdint.h>

class PID {

    public:

        double Kp, Ki, Kd;
        double prev_error;
        double integral;
        bool has_prev;
        uint16_t max_int;
        double p_response, i_response, d_response;

        PID(double Kp, double Ki, double Kd, uint16_t max);
        void reset();
        double seek(double actual, double setpoint, double dt);
};

#endif
