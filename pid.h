#pragma once

class PID
{
public:
    // Kp -  proportional gain
    // Ki -  Integral gain
    // Kd -  derivative gain
    // max - maximum value of manipulated variable
    // min - minimum value of manipulated variable
    PID(float min, float max, float Kp, float Kd, float Ki);

    // Returns the manipulated variable given a setpoint and current process value
    float calculate(float dt, float setpoint, float pv);
    ~PID();

private:
    float _max;
    float _min;
    float _Kp;
    float _Kd;
    float _Ki;
    float _pre_error;
    float _integral;
};
