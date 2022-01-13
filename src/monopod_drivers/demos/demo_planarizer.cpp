/**
 * @file demo_1_motor_print_everything.cpp
 * @copyright Copyright (c) 2018-2020, New York University and Max Planck
 * Gesellschaft, License BSD-3-Clause
 */

#include <tuple>

#include <monopod_sdk/blmc_drivers/devices/can_bus.hpp>
#include <monopod_sdk/blmc_drivers/devices/motor.hpp>
#include <monopod_sdk/blmc_drivers/devices/motor_board.hpp>
#include "monopod_sdk/monopod_drivers/planarizer.hpp"

const int NUMBER_JOINTS = 3;

static THREAD_FUNCTION_RETURN_TYPE plan_printing_loop(void* planarizer_ptr)
{
    // cast input arguments to the right format --------------------------------

    monopod_drivers::Planarizer& planarizer = *(static_cast<monopod_drivers::Planarizer*>(planarizer_ptr));

    while (true)
    {
        auto data = planarizer.get_measurements();
        std::cout << "Current Position for: "
         << "(boom_connector_joint: "   << data[monopod_drivers::position][2] << ")"
         << "(planarizer_yaw_joint: "   << data[monopod_drivers::position][1] << ")"
         << "(planarizer_pitch_joint: " << data[monopod_drivers::position][0] << ")"
         << std::endl;
        // std::cout << "Velocity is: " << data[monopod_drivers::velocity] << std::endl;

    }
    return THREAD_FUNCTION_RETURN_VALUE;
}

int main(int, char**)
{

    auto planarizer = std::make_shared<monopod_drivers::Planarizer>(3, "can1", "can2");

    // // start real-time leg loop --------------------------------------------
    real_time_tools::RealTimeThread plan_printing_thread;
    plan_printing_thread.create_realtime_thread(&plan_printing_loop, &planarizer);


    rt_printf("control loop started \n");
    plan_printing_thread.join();
    return 0;
}
