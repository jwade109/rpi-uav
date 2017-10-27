#include <pid.h>

pid_controller::pid_controller(uint8_t f, double Kp, double Ki,
    double Kd, double max) : freq(f), Kp(Kp), Ki(Ki), Kd(Kd),
    max_int(max), p_response(0), i_response(0), d_response(0),
    error_rate(f), steady_state(f) { }

void pid_controller::reset()
{
    error_rate.reset();
    steady_state.reset();
    p_response = i_response = d_response = 0;
}

double pid_controller::seek(double actual, double setpoint)
{
    double error = setpoint - actual;
    steady_state.step(error);
    steady_state.value = steady_state.value > max_int ? max_int :
        steady_state.value < -max_int ? -max_int : steady_state.value;
    error_rate.step(-actual);

    p_response = Kp * error;
    i_response = Ki * steady_state.value;
    d_response = Kd * error_rate.value;

    return p_response + i_response + d_response;
}
