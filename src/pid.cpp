#include <stdbool.h>
#include <pid.h>

PID::PID(double Kp, double Ki, double Kd, uint16_t max):
    Kp(Kp), Ki(Ki), Kd(Kd), max_int(max)
{
    reset();
}

void PID::reset()
{
    prev_error = 0;
    integral = 0;
    has_prev = false;
    p_response = i_response = d_response = 0;
}

double PID::seek(double actual, double setpoint, double dt)
{
    double error = setpoint - actual, error_rate = 0;
    
    // assign steady state error with windup guards
    integral += error * dt;
    if (integral > max_int)
    {
        integral = max_int;
    }
    else if (integral < -max_int)
    {
        integral = -max_int;
    }
    
    // calculate error_rate if possible
    if (has_prev)
    {
        error_rate = (error - prev_error) / dt;
    }
    
    // store error for calculating error_rate next iteration
    prev_error = error;
    has_prev = true;
    
    p_response = Kp * error;
    i_response = Ki * integral;
    d_response = Kd * error_rate;
    
    return p_response + i_response + d_response;
}
