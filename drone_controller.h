#pragma once

#include <atomic>
#include <chrono>

#ifndef _WIN32
#include <unistd.h>
#endif

#include "multiwii.h"
#include "pid.h"
#include "utils.h"

using namespace std::chrono;

typedef duration<double, std::ratio<1, 1000>> ms;

enum DroneFlightMode : uint32_t
{
    ARM_MODE = (1 << 0),
    HOVER_MODE = (1 << 1),
    FOLLOW_MODE = (1 << 2)
};

const uint LOOP_SLEEP_TIME = 200;
const uint HOVER_TIMEOUT = 3000;

const uint16_t DISABLE_VALUE = 1000;
const uint16_t ENABLE_VALUE = 2000;
const uint16_t MIDDLE_VALUE = 1500;

struct DroneReceiver
{
    uint16_t roll;
    uint16_t pitch;
    uint16_t throttle;
    uint16_t yaw;

    uint16_t flight_mode;
    uint16_t aux_2;
    uint16_t arm_mode;
    uint16_t aux_4;
    uint16_t aux_5;
    uint16_t aux_6;
    uint16_t aux_7;
    uint16_t aux_8;
    uint16_t aux_9;
    uint16_t aux_10;
    uint16_t aux_11;
    uint16_t aux_12;
    uint16_t aux_13;
    uint16_t aux_14;
};

class DroneController
{
public:
    DroneController(std::string msp_port_name);

    void run();

    void update_pid(float dt, float center_x, float center_y, float actual_x, float actual_y);

private:
    std::string msp_port_name;

    static std::atomic<DroneFlightMode> flight_mode;

    PID pid_x;
    PID pid_y;

    static std::atomic<float> dx;
    static std::atomic<float> dy;
};
