#include "drone_controller.h"

DroneController::DroneController(std::string msp_port_name) :
    msp_port_name(msp_port_name),
    pid_x(-100, 100, 0.1, 0.01, 0.5),
    pid_y(-100, 100, 0.1, 0.01, 0.5)
{ }

std::atomic<DroneFlightMode> DroneController::flight_mode{ DroneFlightMode::PENDING_INIT };

std::atomic<float> DroneController::dx{ 0 };
std::atomic<float> DroneController::dy{ 0 };
std::atomic<bool> DroneController::has_detection{ false };

void DroneController::init(size_t cam_width, size_t cam_height)
{
    this->cam_width = cam_width;
    this->cam_height = cam_height;

    slog::info << "Starting drone controller" << slog::endl;

    serial.reset(new ceSerial(msp_port_name, 115200, 8, 'N', 1));

    auto serial_code = (*serial).Open();
    if (serial_code != 0)
    {
        throw std::logic_error("Unable to open serial port");
    }

    flight_mode = DroneFlightMode::RECOVERY_MODE;
}

void DroneController::run()
{
    try
    {
        while (flight_mode == DroneFlightMode::PENDING_INIT)
        {
            std::this_thread::sleep_for(milliseconds(10));
        }

        bool armed = false;
        double throttle = DISABLE_VALUE;

        auto t0 = high_resolution_clock::now();

        // 1. Arm
        //   a. Keep sending 0 throttle and disable arm
        // 2. Hover up until y is good
        // 3. Begin follow procedure

        high_resolution_clock::time_point fly_up_begin;

        while (true)
        {
            Msp::MspStatusEx* rcv = Msp::receive_parameters<Msp::MspStatusEx>(&*serial, Msp::MspCommand::STATUS_EX);
            auto arming_flags = rcv->arming_flags;

            // printf("\nflight_mode_flags: %u, average_system_load_percent: %u, arming_flags: %u \n", rcv->initial_flight_mode_flags, rcv->average_system_load_percent, rcv->arming_flags);

            // for (unsigned int i = 0; i < 22; i++)
            // {
            //     if (arming_flags & (1u << i))
            //         slog::info << " " << (i + 1) << ": " << ((arming_flags & (1u << i)) == (1u << i));
            // }

            if ((arming_flags & Msp::MspArmingDisableFlags::ARMING_DISABLED_RX_FAILSAFE)
                || (arming_flags & Msp::MspArmingDisableFlags::ARMING_DISABLED_FAILSAFE))
            {
                // If the drone loses RX and it's still armed then it will go into failsafe mode
                // To get out of failsafe, we need to reset the throttle and disable the arming switch
                // for 2 seconds before trying to do anything else

                flight_mode = DroneFlightMode::RECOVERY_MODE;
            }
            else if (flight_mode == DroneFlightMode::RECOVERY_MODE)
            {
                flight_mode = DroneFlightMode::FLY_UP_MODE;
                fly_up_begin = high_resolution_clock::now();
            }

            // --------------------------------------------

            if (flight_mode == DroneFlightMode::FLY_UP_MODE)
            {
                double ms_of_detection = 0;

                auto t_init = high_resolution_clock::now();
                auto t0 = high_resolution_clock::now();

                while (true)
                {
                    auto now = high_resolution_clock::now();
                    auto dt = duration_cast<ms>(now - t0).count();
                    auto elapsed = duration_cast<ms>(now - t_init).count();

                    std::cout << "\nElapsed time: " << elapsed << std::endl;

                    t0 = now;

                    // If we have 10ms of constant detection, switch to hover mode
                    if (elapsed >= HOVER_TIMEOUT || ms_of_detection >= 10)
                        break;

                    if (has_detection)
                        ms_of_detection += dt;
                    else
                        ms_of_detection = 0;

                    // Otherwise, linearly increase throttle to max (1050)
                    throttle = DISABLE_VALUE + min(elapsed / 4.0, 50.0);
                    send_throttle_command((uint16_t)round(throttle));

                    std::this_thread::sleep_for(milliseconds(1));
                }

                flight_mode = DroneFlightMode::HOVER_MODE;
            }

            switch (flight_mode)
            {
                case DroneFlightMode::RECOVERY_MODE:
                {
                    armed = false;
                    throttle = DISABLE_VALUE;
                    break;
                }
                case DroneFlightMode::HOVER_MODE:
                {
                    armed = false;
                    break;
                }
            }

            DroneReceiver params = {
                .roll        = MIDDLE_VALUE,
                .pitch       = MIDDLE_VALUE,
                .throttle    = ((uint16_t)round(throttle)),
                .yaw         = MIDDLE_VALUE,
                .flight_mode = DISABLE_VALUE, // DISABLE = Horizon mode
                .aux_2       = MIDDLE_VALUE,
                .arm_mode    = armed ? ENABLE_VALUE : DISABLE_VALUE
            };
            Msp::send_command<DroneReceiver>(&*serial, Msp::MspCommand::SET_RAW_RC, &params);

            auto t1 = std::chrono::high_resolution_clock::now();
            auto time_diff = std::chrono::duration_cast<ms>(t1 - t0).count();

            //std::cout << "\nCommand time: " << time_diff << std::endl;

            t0 = t1;

            std::this_thread::sleep_for(milliseconds(LOOP_SLEEP_TIME));
        }
    }
    catch (const std::exception& error)
    {
        slog::err << error.what() << slog::endl;
        return;
    }
    catch (...)
    {
        slog::err << "Unknown/internal exception happened." << slog::endl;
        return;
    }
}

void DroneController::send_throttle_command(uint16_t throttle)
{
    DroneReceiver params = {
        .roll        = MIDDLE_VALUE,
        .pitch       = MIDDLE_VALUE,
        .throttle    = throttle,
        .yaw         = MIDDLE_VALUE,
        .flight_mode = DISABLE_VALUE, // DISABLE = Horizon mode
        .aux_2       = MIDDLE_VALUE,
        .arm_mode    = ENABLE_VALUE
    };
    Msp::send_command<DroneReceiver>(&*serial, Msp::MspCommand::SET_RAW_RC, &params);
}

// This method is being run from the inference thread so we don't need PID to be thread safe,
// only need dx and dy to be thread safe.
void DroneController::update_pid(double dt, double actual_x, double actual_y, float size)
{
    has_detection = actual_x != -1 && actual_y != -1 && size != -1;

    if (has_detection && (flight_mode & (DroneFlightMode::HOVER_MODE | DroneFlightMode::FOLLOW_MODE)))
    {
        // TODO: Wait a couple seconds first

        dy = (float)pid_y.calculate(dt, cam_height / 2.0, actual_y);

        if (flight_mode & DroneFlightMode::FOLLOW_MODE)
        {
            dx = (float)pid_x.calculate(dt, cam_width / 2.0, actual_x);
        }
    }

    std::cout << "Adj: " << cv::Point2f(dx, dy) << std::endl;
}
