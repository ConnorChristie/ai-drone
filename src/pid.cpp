#include "pid.h"

using namespace std;

PID::PID(double min, double max, double Kp, double Kd, double Ki) :
    _max(max),
    _min(min),
    _Kp(Kp),
    _Kd(Kd),
    _Ki(Ki),
    _pre_error(0),
    _integral(0)
{
}

double PID::calculate(double dt, double setpoint, double pv)
{
    // Calculate error
    auto error = setpoint - pv;

    // Proportional term
    auto Pout = _Kp * error;

    // Integral term
    _integral += error * dt;
    auto Iout = _Ki * _integral;

    // Derivative term
    auto derivative = (error - _pre_error) / dt;
    auto Dout = _Kd * derivative;

    // Calculate total output
    auto output = Pout + Iout + Dout;

    // Restrict to max/min
    if (output > _max)
        output = _max;
    else if (output < _min)
        output = _min;

    // Save error to previous error
    _pre_error = error;

    return output;
}

PID::~PID()
{
}
