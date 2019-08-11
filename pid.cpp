#include <iostream>
#include <cmath>
#include "pid.h"

using namespace std;

PID::PID(float min, float max, float Kp, float Kd, float Ki) :
    _max(max),
    _min(min),
    _Kp(Kp),
    _Kd(Kd),
    _Ki(Ki),
    _pre_error(0),
    _integral(0)
{
}

float PID::calculate(float dt, float setpoint, float pv)
{
    // Calculate error
    float error = setpoint - pv;

    // Proportional term
    float Pout = _Kp * error;

    // Integral term
    _integral += error * dt;
    float Iout = _Ki * _integral;

    // Derivative term
    float derivative = (error - _pre_error) / dt;
    float Dout = _Kd * derivative;

    // Calculate total output
    float output = Pout + Iout + Dout;

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
