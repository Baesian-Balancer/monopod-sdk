/**
 * @file demo_sine_position_1_motor.cpp
 * @copyright Copyright (c) 2018-2020, New York University and Max Planck
 * Gesellschaft, License BSD-3-Clause
 */

#include "monopod_sdk/monopod_drivers/leg.hpp"
#include "sine_position_control.hpp"
#include <atomic>
#include <signal.h>

/**
 * @brief This boolean is here to kill cleanly the application upon ctrl+c
 */
std::atomic_bool StopDemos(false);

/**
 * @brief This function is the callback upon a ctrl+c call from the terminal.
 *
 * @param s
 */
void my_handler(int) { StopDemos = true; }

/**
 * @brief This is the main demo program.
 *
 * @param argc
 * @param argv
 * @return int
 */
int main(int, char **) {
  // make sure we catch the ctrl+c signal to kill the application properly.
  struct sigaction sigIntHandler;
  sigIntHandler.sa_handler = my_handler;
  sigemptyset(&sigIntHandler.sa_mask);
  sigIntHandler.sa_flags = 0;
  sigaction(SIGINT, &sigIntHandler, NULL);
  StopDemos = false;

  monopod_drivers::sdk_Ptr sdk = std::make_shared<monopod_drivers::Monopod>();
  sdk->initialize(monopod_drivers::Mode::motor_board);
  sdk->calibrate(1, 1);

  rt_printf("sdk is set up \n");

  // construct a simple PD controller.
  monopod_drivers::SinePositionControl controller(sdk);

  rt_printf("controllers are set up \n");

  controller.start_loop();

  rt_printf("loops have started \n");

  // Wait until the application is killed.
  while (!StopDemos) {
    real_time_tools::Timer::sleep_sec(0.01);
  }

  controller.stop_loop();

  return 0;
}
