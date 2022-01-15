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
    rt_printf("initialized monopod sdk \n");

    monopod.start_loop();
    rt_printf("loops have started \n");


    while (!StopDemos)
    {
        real_time_tools::Timer::sleep_sec(1);

        std::vector<double> poss = monopod.get_positions().value();
        std::cout << "Joint Positions: ";
        for (const auto &pos: poss){
            std::cout << pos << " ";
        }
        std::cout << std::endl << std::endl;

    }

    return 0;
}
