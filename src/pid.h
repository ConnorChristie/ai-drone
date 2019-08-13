#pragma once

#include <atomic>
#include <iostream>
#include <cmath>

class PID
{
public:
    // Kp -  proportional gain
    // Ki -  Integral gain
    // Kd -  derivative gain
    // max - maximum value of manipulated variable
    // min - minimum value of manipulated variable
    PID(double min, double max, double Kp, double Kd, double Ki);

    // Returns the manipulated variable given a setpoint and current process value
    double calculate(double dt, double setpoint, double pv);
    ~PID();

private:
    double _max;
    double _min;
    double _Kp;
    double _Kd;
    double _Ki;

    double _pre_error;
    double _integral;
};
