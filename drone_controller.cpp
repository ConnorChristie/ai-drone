#include "drone_controller.h"

DroneController::DroneController(std::string msp_port_name) :
    msp_port_name(msp_port_name)
{ }

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

    bool in_rx_recovery = true;

    typedef std::chrono::duration<double, std::ratio<1, 1000>> ms;
    auto t0 = std::chrono::high_resolution_clock::now();

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

        if (in_rx_recovery)
        {
            Msp::MspReceiver reset_params = {
                .roll = 1500,
                .pitch = 1500,
                .throttle = 900,
                .yaw = 1500,
                .aux_1 = 1000,
                .aux_2 = 1000,
                .aux_3 = 1000
            };
            Msp::send_command<Msp::MspReceiver>(serial, Msp::MspCommand::SET_RAW_RC, &reset_params);
        }
        else
        {
            uint16_t new_throttle = throttle + 900;

            Msp::MspReceiver params = {
                .roll = 1500,
                .pitch = 1500,
                .throttle = new_throttle,
                .yaw = 1500,
                .aux_1 = 1000,
                .aux_2 = 1000,
                .aux_3 = 2000
            };
            Msp::send_command<Msp::MspReceiver>(serial, Msp::MspCommand::SET_RAW_RC, &params);

            throttle = (throttle + 1) % 200;
        }

        // If the drone loses RX and it's still armed then it will go into failsafe mode
        // To get out of failsafe, we need to reset the throttle and disable the arming switch
        // for 2 seconds before trying to do anything else

        in_rx_recovery =
            (arming_flags & Msp::MspArmingDisableFlags::ARMING_DISABLED_RX_FAILSAFE)
            || (arming_flags & Msp::MspArmingDisableFlags::ARMING_DISABLED_FAILSAFE);

        auto t1 = std::chrono::high_resolution_clock::now();
        auto time_diff = std::chrono::duration_cast<ms>(t1 - t0).count();

        //std::cout << "\nCommand time: " << time_diff << std::endl;

        t0 = t1;

        CALL(Sleep, usleep, 200);
    }
}