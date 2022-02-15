#include <math.h>
#include <monopod_sdk/monopod.hpp>

#include <atomic>
#include <fstream>
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

int main(int, char **) {
  // make sure we catch the ctrl+c signal to kill the application properly.
  struct sigaction sigIntHandler;
  sigIntHandler.sa_handler = my_handler;
  sigemptyset(&sigIntHandler.sa_mask);
  sigIntHandler.sa_flags = 0;
  sigaction(SIGINT, &sigIntHandler, NULL);
  StopDemos = false;

  monopod_drivers::Monopod monopod;
  rt_printf("controllers are set up \n");

  // monopod.initialize(monopod_drivers::Mode::FIXED_CONNECTOR);
  monopod.initialize(monopod_drivers::Mode::MOTOR_BOARD);
  rt_printf("initialized monopod sdk \n");

  real_time_tools::Timer time_logger;

  while (!StopDemos) {
    // measure the time spent.
    time_logger.tac();
    // Printings
    rt_printf("\33[H\33[2J"); // clear screen
    monopod.print();          // print info

    auto input_data = monopod.get_positions().value();
    std::cout << "Positions from SDK: ";
    for (auto data : input_data) {
      std::cout << data << ", ";
    }
    std::cout << std::endl;

    time_logger.print_statistics();
    fflush(stdout);
    real_time_tools::Timer::sleep_sec(0.5);

  } // endwhile

  time_logger.dump_measurements("/tmp/demo_print_sdk");

  return 0;
}
