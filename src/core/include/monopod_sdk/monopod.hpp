#pragma once

#include <time_series/time_series.hpp>
#include "real_time_tools/spinner.hpp"
#include "real_time_tools/timer.hpp"
#include <real_time_tools/thread.hpp>

#include <math.h>
#include <fstream>
#include <mutex>
#include <unordered_map>
#include <optional>


namespace monopod_drivers
{

/**
 * @brief This is a basic PD controller to be used in the demos of this package.
 */
class Monopod
{

public:

    /**
    * @brief Joint names indexed same as enumerator
    */
    const std::vector<std::string> joint_names = {
        "hip_joint",
        "knee_joint",
        "boom_connector_joint",
        "planarizer_yaw_joint",
        "planarizer_pitch_joint"
    };

private:

    /**
    * @brief ScalarTimeseries is a simple shortcut for more intelligible code.
    */
    typedef time_series::TimeSeries<double> ScalarTimeseries;

    /**
     * @brief the realt time thread object.
     */
    real_time_tools::RealTimeThread rt_thread_;

    /**
     * @brief managing the stopping of the loop
     */
    bool stop_loop;

    /**
     * @brief Enumerates the joint names for indexing
     */
    enum JointNameIndexing
    {
       hip_joint,
       knee_joint,
       boom_connector_joint,
       planarizer_yaw_joint,
       planarizer_pitch_joint
    };

    /**
    * @brief Joint names indexed same as enumerator for actuators
    */
    std::vector<int>  motor_joint_indexing = {hip_joint, knee_joint};

    struct
    {

        /**
         * @brief Mutex lock for write buffer
         */
        std::mutex write_door;
        std::optional<std::vector<std::string>> scopedJointNames;

        /**
         * @brief Write Buffer
         */
        using JointWrite = double;
        std::unordered_map<JointNameIndexing, JointWrite> write = {
            {hip_joint, 0.0},
            {knee_joint, 0.0}
        };

        /**
         * @brief Mutex lock for read buffer
         */
        std::mutex read_door;

        /**
         * @brief Read Buffer
         */
        using JointRead = double;
        std::unordered_map<JointNameIndexing, JointRead> read = {
            {planarizer_pitch_joint, 0.0},
            {planarizer_yaw_joint, 0.0},
            {boom_connector_joint, 0.0},
            {hip_joint, 0.0},
            {knee_joint, 0.0}
        };

    } buffers;





public:

    /**
     * @brief Construct a new Monopod object.
     *
     * @param motor_slider_pairs
     */
    Monopod();

    /**
     * @brief Destroy the Monopod object
     */
    ~Monopod();

    /**
    * @brief Initialize canbus connections to encoder board and motor board.
    */
    bool initialize();

    /**
     * @brief This method is a helper to start the thread loop.
     */
    void start_loop();

    /**
    * @brief Set the torque target for some joint index. Return the bool whether success
    */
    bool set_torque_target(const double &torque_target, const int joint_index);

    /**
    * @brief Set the torque targets for all joint indexes. Return the bool whether success
    */
    bool set_torque_targets(const std::vector<double> &torque_targets, const std::vector<int> &joint_indexes={});

private:

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
     * @brief this is a simple control loop which runs at a kilohertz.
     *
     * Gets indees on joint index enum
     */


};  // end class Monopod definition

}  // namespace monopod_drivers
