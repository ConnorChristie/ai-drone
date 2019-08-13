#pragma once

#include <atomic>
#include <chrono>
#include <thread>

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
    PENDING_INIT  = (1 << 0),
    RECOVERY_MODE = (1 << 1),
    FLY_UP_MODE   = (1 << 2),
    HOVER_MODE    = (1 << 3),
    FOLLOW_MODE   = (1 << 4)
};

const uint LOOP_SLEEP_TIME = 2;
const uint HOVER_TIMEOUT   = 5000;

const uint16_t DISABLE_VALUE = 1000;
const uint16_t ENABLE_VALUE  = 2000;
const uint16_t MIDDLE_VALUE  = 1500;

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

    void init(size_t cam_width, size_t cam_height);
    void run();

    void update_pid(double dt, double actual_x, double actual_y, float size);

private:
    shared_ptr<ceSerial> serial;
    std::string msp_port_name;

    size_t cam_width;
    size_t cam_height;

    static std::atomic<DroneFlightMode> flight_mode;

    PID pid_x;
    PID pid_y;

    static std::atomic<float> dx;
    static std::atomic<float> dy;
    static std::atomic<bool> has_detection;

    void send_throttle_command(uint16_t throttle);
};
