#pragma once

#define _USE_MATH_DEFINES

#include <atomic>
#include <chrono>
#include <thread>
#include <cmath>

#ifndef _WIN32
#include <unistd.h>
#endif

#include <opencv2/opencv.hpp>
#include "multiwii.h"
#include "pid.h"

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

constexpr uint16_t LOOP_SLEEP_TIME = 2;
constexpr uint16_t HOVER_TIMEOUT   = 10000;

constexpr uint16_t MIDDLE_VALUE  = 1500;
constexpr uint16_t DISABLE_VALUE = MIDDLE_VALUE - 512;
constexpr uint16_t ENABLE_VALUE  = MIDDLE_VALUE + 512;

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

    size_t cam_width = 0;
    size_t cam_height = 0;

    static std::atomic<DroneFlightMode> flight_mode;

    PID pid_x;
    PID pid_y;

    static std::atomic<float> dx;
    static std::atomic<float> dy;
    static std::atomic<bool> has_detection;

    void run_fly_up_procedure();
    void send_throttle_command(uint16_t throttle);
};
