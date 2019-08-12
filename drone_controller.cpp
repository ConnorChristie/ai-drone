#include "drone_controller.h"

DroneController::DroneController(std::string msp_port_name) :
    msp_port_name(msp_port_name),
    pid_x(-100, 100, 0.1, 0.01, 0.5),
    pid_y(-100, 100, 0.1, 0.01, 0.5)
{ }

std::atomic<DroneFlightMode> DroneController::flight_mode{ DroneFlightMode::ARM_MODE };

std::atomic<float> DroneController::dx{ 0 };
std::atomic<float> DroneController::dy{ 0 };

void DroneController::run()
{
    slog::info << "Start drone controller" << slog::endl;

    ceSerial* serial = new ceSerial(msp_port_name, 115200, 8, 'N', 1);

    auto serial_code = serial->Open();
    if (serial_code != 0)
    {
        printf("Unable to open serial port: %li\n", serial_code);
        return;
    }

    uint16_t throttle = 0;

    auto t0 = high_resolution_clock::now();

    // 1. Arm
    //   a. Keep sending 0 throttle and disable arm
    // 2. Hover up until y is good
    // 3. Begin follow procedure

    high_resolution_clock::time_point hover_begin;

    while (true)
    {
        Msp::MspStatusEx* rcv = Msp::receive_parameters<Msp::MspStatusEx>(serial, Msp::MspCommand::STATUS_EX);
        printf("\nflight_mode_flags: %u, average_system_load_percent: %u, arming_flags: %u \n", rcv->initial_flight_mode_flags, rcv->average_system_load_percent, rcv->arming_flags);

        uint32_t arming_flags = rcv->arming_flags;
        for (unsigned int i = 0; i < 22; i++)
        {
            if (arming_flags & (1u << i))
                slog::info << " " << (i + 1) << ": " << ((arming_flags & (1u << i)) == (1u << i));
        }

        if ((arming_flags & Msp::MspArmingDisableFlags::ARMING_DISABLED_RX_FAILSAFE)
            || (arming_flags & Msp::MspArmingDisableFlags::ARMING_DISABLED_FAILSAFE))
        {
            // If the drone loses RX and it's still armed then it will go into failsafe mode
            // To get out of failsafe, we need to reset the throttle and disable the arming switch
            // for 2 seconds before trying to do anything else

            flight_mode = DroneFlightMode::ARM_MODE;
        }
        else if (flight_mode == DroneFlightMode::ARM_MODE)
        {
            flight_mode = DroneFlightMode::HOVER_MODE;
            hover_begin = high_resolution_clock::now();
        }
        else if (flight_mode == DroneFlightMode::HOVER_MODE)
        {
            auto t1 = high_resolution_clock::now();
            auto time_diff = duration_cast<ms>(t1 - hover_begin).count();

            // 1. Increase throttle until we reach correct Y height (follow PID)
            // 2. Timeout after a couple of seconds then decrease throttle

            if (time_diff >= HOVER_TIMEOUT)
            {

            }
        }

        if (flight_mode == DroneFlightMode::ARM_MODE)
        {
            DroneReceiver reset_params = {
                .roll        = MIDDLE_VALUE,
                .pitch       = MIDDLE_VALUE,
                .throttle    = DISABLE_VALUE,
                .yaw         = MIDDLE_VALUE,
                .flight_mode = DISABLE_VALUE,
                .aux_2       = MIDDLE_VALUE,
                .arm_mode    = DISABLE_VALUE
            };
            Msp::send_command<DroneReceiver>(serial, Msp::MspCommand::SET_RAW_RC, &reset_params);
        }

        if (flight_mode == DroneFlightMode::HOVER_MODE)
        {
            uint16_t new_throttle = throttle + DISABLE_VALUE;

            DroneReceiver params = {
                .roll        = MIDDLE_VALUE,
                .pitch       = MIDDLE_VALUE,
                .throttle    = new_throttle,
                .yaw         = MIDDLE_VALUE,
                .flight_mode = DISABLE_VALUE,
                .aux_2       = MIDDLE_VALUE,
                .arm_mode    = ENABLE_VALUE
            };
            Msp::send_command<DroneReceiver>(serial, Msp::MspCommand::SET_RAW_RC, &params);

            throttle = (throttle + 1) % 200;
        }

        auto t1 = std::chrono::high_resolution_clock::now();
        auto time_diff = std::chrono::duration_cast<ms>(t1 - t0).count();

        //std::cout << "\nCommand time: " << time_diff << std::endl;

        t0 = t1;

        CALL(Sleep, usleep, LOOP_SLEEP_TIME);
    }
}

// This method is being run from the inference thread so we don't need PID to be thread safe,
// only need dx and dy to be thread safe.
void DroneController::update_pid(float dt, float center_x, float center_y, float actual_x, float actual_y)
{
    if (flight_mode != DroneFlightMode::ARM_MODE)
    {
        // TODO: Wait a couple seconds first

        dy = pid_y.calculate(dt, center_y, actual_y);
    }

    std::cout << "Adj: " << cv::Point2f(dx, dy) << std::endl;
}
