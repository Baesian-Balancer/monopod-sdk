#include <math.h>
#include <monopod_sdk/monopod.hpp>

#include <signal.h>
#include <atomic>


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


int main(int, char**)
{
    // make sure we catch the ctrl+c signal to kill the application properly.
    struct sigaction sigIntHandler;
    sigIntHandler.sa_handler = my_handler;
    sigemptyset(&sigIntHandler.sa_mask);
    sigIntHandler.sa_flags = 0;
    sigaction(SIGINT, &sigIntHandler, NULL);
    StopDemos = false;

    monopod_drivers::Monopod monopod;
    rt_printf("controllers are set up \n");

    monopod.initialize();
    monopod.start_loop();
    rt_printf("loops have started \n");
    double x = 0;


    while (!StopDemos)
    {
        real_time_tools::Timer::sleep_sec(1);

        // monopod.set_torque_target(x, 0);
        // monopod.set_torque_target(x + 0.69, 1);
        // monopod.set_torque_targets({x, x + 1});

        std::vector<double> poss = monopod.get_velocities().value();
        for (auto i: poss)
            std::cout << i << ", ";
        std::cout << std::endl << std::endl;
        x++;
    }

    return 0;
}
