#pragma once

#ifndef _WIN32
#include <unistd.h>
#endif

#include "multiwii.h"
#include "utils.h"

class DroneController
{
public:
    DroneController(std::string msp_port_name);

    void run();

private:
    std::string msp_port_name;
};
