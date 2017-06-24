#ifndef PID_LIB_H
#define PID_LIB_H

#include <inttypes.h>
#include <stdbool.h>

typedef struct
{
    double Kp; // proportional gain
    double Ki; // integral gain
    double Kd; // derivative gain
    double prev_error;
    double integral;
    double max_int;
    
    double prev_p_response;
    double prev_i_response;
    double prev_d_response;
    
    bool has_prev;
}
Controller;

Controller pid_init(double Kp, double Ki, double Kd, double max);

void pid_zero(Controller* c);

double pid_seek(Controller* c, double actual, double setpoint, double time);

#endif
