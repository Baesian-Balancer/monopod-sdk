#include <math.h>
#include "real_time_tools/spinner.hpp"
#include "real_time_tools/timer.hpp"

namespace monopod_drivers
{

/**
 * @brief This is a basic PD controller to be used in the demos of this package.
 */
class Monopod
{
public:
    /**
     * @brief Construct a new Monopod object.
     *
     * @param motor_slider_pairs
     */
    Monopod()
    {
        stop_loop = false;
    }

    /**
     * @brief Destroy the Monopod object
     */
    ~Monopod()
    {
        stop_loop = true;
        rt_thread_.join();
    }

    /**
     * @brief This method is a helper to start the thread loop.
     */
    void start_loop()
    {
        rt_thread_.create_realtime_thread(&Monopod::loop, this);
    }

private:
    /**
     * @brief This is the real time thread object.
     */
    real_time_tools::RealTimeThread rt_thread_;

    /**
     * @brief this function is just a wrapper around the actual loop function,
     * such that it can be spawned as a posix thread.
     */
    static THREAD_FUNCTION_RETURN_TYPE loop(void* instance_pointer)
    {
        ((Monopod*)(instance_pointer))->loop();
        return THREAD_FUNCTION_RETURN_VALUE;
    }

    /**
     * @brief this is a simple control loop which runs at a kilohertz.
     *
     * it reads the measurement from the analog sensor, in this case the
     * slider. then it scales it and sends it as the current target to
     * the motor.
     */
    void loop();

    /**
     * @brief managing the stopping of the loop
     */
    bool stop_loop;

};  // end class Monopod definition

}  // namespace blmc_drivers
