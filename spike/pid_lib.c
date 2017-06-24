#include "pid_lib.h"
#include <stdio.h>

Controller pid_init(double Kp, double Ki, double Kd, double max)
{
    Controller c;
    c.Kp = Kp;
    c.Ki = Ki;
    c.Kd = Kd;
    c.prev_error = 0;
    c.integral = 0;
    c.has_prev = 0;
    if (max > 0) c.max_int = max;
    else c.max_int = 9999999;
    return c;
}

void pid_zero(Controller* c)
{
    c->prev_error = 0;
    c->integral = 0;
    c->has_prev = 0;
}

double pid_seek(Controller* c, double actual, double setpoint, double dt)
{
    double error = setpoint - actual, error_rate = 0;
    
    // assign steady state error with windup guards
    c->integral += error * dt;
    if (c->integral > c->max_int)
    {
        c->integral = c->max_int;
    }
    else if (c->integral < -(c->max_int))
    {
        c->integral = -(c->max_int);
    }
    
    // calculate error_rate if possible
    if (c->has_prev)
    {
        error_rate = (error - c->prev_error) / dt;
    }
    
    // store error for calculating error_rate next iteration
    c->prev_error = error;
    c->has_prev = 1;
    
    c->prev_p_response = c->Kp * error;
    c->prev_i_response = c->Ki * c->integral;
    c->prev_d_response = c->Kd * error_rate;
    
    return c->prev_p_response + c->prev_i_response + c->prev_d_response;
}
