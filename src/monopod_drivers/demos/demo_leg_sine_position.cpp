/**
 * @file demo_sine_position_1_motor.cpp
 * @copyright Copyright (c) 2018-2020, New York University and Max Planck
 * Gesellschaft, License BSD-3-Clause
 */

#include <signal.h>
#include <atomic>
#include "sine_position_control.hpp"
#include "monopod_sdk/monopod_drivers/leg.hpp"

/**
 * @brief Defines a static Eigen vector type in order to define the
 * interface. Two is for number of joints
 */
typedef Eigen::Matrix<double, 2, 1> Vector;

/**
 * @brief This boolean is here to kill cleanly the application upon ctrl+c
 */
std::atomic_bool StopDemos(false);

/**
 * @brief This function is the callback upon a ctrl+c call from the terminal.
 *
 * @param s
 */
void my_handler(int)
{
    StopDemos = true;
}

/**
 * @brief This is the main demo program.
 *
 * @param argc
 * @param argv
 * @return int
 */
int main(int, char **)
{
    // make sure we catch the ctrl+c signal to kill the application properly.
    struct sigaction sigIntHandler;
    sigIntHandler.sa_handler = my_handler;
    sigemptyset(&sigIntHandler.sa_mask);
    sigIntHandler.sa_flags = 0;
    sigaction(SIGINT, &sigIntHandler, NULL);
    StopDemos = false;

    auto can_bus_ = std::make_shared<monopod_drivers::CanBus>("can0");
    auto board_ = std::make_shared<monopod_drivers::CanBusMotorBoard>(can_bus_);

    std::shared_ptr<monopod_drivers::Leg> leg = std::make_shared<monopod_drivers::Leg>(board_);
    leg->initialize();

    rt_printf("leg is set up \n");

    // construct a simple PD controller.
    monopod_drivers::SinePositionControl controller(leg);

    rt_printf("controllers are set up \n");

    controller.start_loop();

    rt_printf("loops have started \n");

    // Wait until the application is killed.
    while (!StopDemos)
    {
        real_time_tools::Timer::sleep_sec(0.01);
    }

    controller.stop_loop();

    return 0;
}
