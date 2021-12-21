#pragma once

#include <time_series/time_series.hpp>
#include <real_time_tools/spinner.hpp>
#include <real_time_tools/timer.hpp>
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
    * @brief Construct a new Monopod object.
    *
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
    * @brief Get model name
    *
    * @return String of model name
    */
    std::string get_model_name() const;

    /**
    * @brief Get a map of joint strings indexing their enumerator index
    *
    * @return Unordered map of joint name strings as key and index as value
    */
    std::unordered_map<std::string, int> get_joint_names() const;

    // ======================================
    // setters
    //=======================================

    /**
    * @brief Set the torque target for some joint index. Return the bool whether success
    *
    * @param torque_target is the desired torque target for indexed joint
    * @param joint_index name of the joint we want to access
    * @return bool whether setting the value was successfull
    */
    bool set_torque_target(const double &torque_target, const int joint_index);

    /**
    * @brief Set the torque targets for all joint indexes. Return the bool whether success
    *
    * @param torque_targets vector of desired torque targets for indexed joints
    * @param joint_indexes names of the joints we want to access
    * @return bool whether setting the value was successfull
    */
    bool set_torque_targets(const std::vector<double> &torque_targets, const std::vector<int> &joint_indexes={});

    // ======================================
    // getters
    //=======================================

    /**
    * @brief Get the torque
    *
    * @param joint_index
    * @return std::optional<double> containing the torque if success
    */
    std::optional<double> get_torque_target(const int &joint_index);

    /**
    * @brief Get the torques of indexed joints
    *
    * @param joint_index
    * @return std::optional<double> containing the torque if success
    */
    std::optional<std::vector<double>> get_torque_targets(const std::vector<int> &joint_indexes={});

    /**
    * @brief Get the position of joint
    *
    * @param joint_index name of the joint we want to access
    * @return std::optional<double> containing the position if success
    * value of the position (NULL if not valid)
    */
    std::optional<double> get_position(const int &joint_index);

    /**
    * @brief Get the velocity of the joint
    *
    * @param joint_index name of the joint we want to access
    * @return std::optional<double> containing the velocity if success
    */
    std::optional<double> get_velocity(const int &joint_index);

    /**
    * @brief Get the acceleration of the joint
    *
    * @param joint_index name of the joint we want to access
    * @return std::optional<double> containing the acceleration if success
    */
    std::optional<double> get_acceleration(const int &joint_index);

    /**
    * @brief Get the position of the joint indexes
    *
    * @param joint_indexes names of the joints we want to access
    * @return std::optional<vector<double>> containing vector of positions if success
    */
    std::optional<std::vector<double>> get_positions(const std::vector<int> &joint_indexes={});

    /**
    * @brief Get the velocity of the joint indexes
    *
    * @param joint_indexes names of the joints we want to access
    * @return std::optional<std::vector<double>> containing vector of velocities if success
    */
    std::optional<std::vector<double>> get_velocities(const std::vector<int> &joint_indexes={});

    /**
    * @brief Get the acceleration of the joint indexes
    *
    * @param joint_indexes names of the joints we want to access
    * @return std::optional<std::vector<double>> containing vector of accelerations if success
    */
    std::optional<std::vector<double>> get_accelerations(const std::vector<int> &joint_indexes={});


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
    * @brief Simple helper method to serialized getting of data.
    *
    * Gets indees on joint index enum
    */

    struct JointReadState; //Forward declaration

    std::optional<std::vector<double>> getJointDataSerialized(
    const Monopod* monopod,
    const std::vector<int>& joint_indexes,
    std::function<double(JointReadState)> getJointData)
    {
        // Take the joint index in lambda. Return the data you want.
        const std::vector<int>& jointSerialization =
          joint_indexes.empty() ? monopod->encoder_joint_indexing : joint_indexes;

        std::vector<double> data;
        data.reserve(jointSerialization.size());
        buffers.read_door.lock();
        for (auto& joint_index : jointSerialization) {
            switch(joint_index)
            {
                case hip_joint:
                case knee_joint:
                case boom_connector_joint:
                case planarizer_yaw_joint:
                case planarizer_pitch_joint:
                    data.push_back(getJointData(buffers.read[(JointNameIndexing)joint_index]));
                    break;
                default:
                    buffers.read_door.unlock();
                    return std::nullopt;

            }
        }
        buffers.read_door.unlock();
        return data;
    }

public:

   /**
   * @brief Joint names indexed same as enumerator
   */
   const std::unordered_map<std::string, int> joint_names = {
       {"hip_joint", hip_joint},
       {"knee_joint", knee_joint},
       {"boom_connector_joint", boom_connector_joint},
       {"planarizer_yaw_joint", planarizer_yaw_joint},
       {"planarizer_pitch_joint", planarizer_pitch_joint}
   };

private:

   /**
    * @brief the realt time thread object.
    */
   real_time_tools::RealTimeThread rt_thread_;

   /**
    * @brief managing the stopping of the loop
    */
   bool stop_loop;

    /**
    * @brief MotorMeasurementIndexing this enum allow to access the different
    * kind of sensor measurements in an understandable way in the code.
    *
    * Note: This is same as in leg.hpp for consistency
    */
    enum MotorMeasurementIndexing
    {
      current,
      position,
      velocity,
      encoder_index,
      motor_measurement_count
    };

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
    * @brief Write Joint names indexed same as enumerator for actuators
    */
    std::vector<int>  motor_joint_indexing =
    {
      hip_joint,
      knee_joint
    };

   /**
   * @brief Read Joint names indexed same as enumerator for encoders
   */
   std::vector<int>  encoder_joint_indexing =
    {
      hip_joint,
      knee_joint,
      boom_connector_joint,
      planarizer_yaw_joint,
      planarizer_pitch_joint
    };

   /**
   * @brief Structure holding the observed state of a joint
   */
   struct JointReadState
   {
       JointReadState() = default;
       JointReadState(const double _pos, const double _vel, const double _acc)
           : pos(_pos), vel(_vel), acc(_acc)
           {}
       double pos = 0.0;
       double vel = 0.0;
       double acc = 0.0;
   };

   /**
   * @brief Structure holding joint limits
   */
   struct JointLimit
   {
       JointLimit()
       {
           constexpr double m = std::numeric_limits<double>::lowest();
           constexpr double M = std::numeric_limits<double>::max();

           min = m;
           max = M;
       }

       JointLimit(const double _min, const double _max)
           : min(_min), max(_max)
       {}

       double min;
       double max;
   };

   /**
   * @brief Structure holding the observed state of a joint
   */
   struct JointSettingsState
   {
       JointSettingsState() = default;
       JointSettingsState(const double _max_torque_target, const JointLimit _position_limit,
         const JointLimit _velocity_limit, const JointLimit _acceleration_limit,
         const double _p, const double _i, const double _d) :
            position_limit(_position_limit), velocity_limit(_velocity_limit),
            acceleration_limit(_acceleration_limit), max_torque_target(_max_torque_target),
            p(_p), i(_i), d(_d)
       {}

       JointLimit position_limit = {};
       JointLimit velocity_limit = {};
       JointLimit acceleration_limit = {};
       double max_torque_target = 0;
       double p = 0;
       double i = 0;
       double d = 0;
   };



   /**
    * @brief Read/Write buffer
    */
   struct buffers
   {
       /**
        * @brief Mutex lock for write buffer
        */
       std::mutex write_door;

       /**
        * @brief Write Buffer
        */
       using JointWriteState = double;
       std::unordered_map<JointNameIndexing, JointWriteState> write = {
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
       std::unordered_map<JointNameIndexing, JointReadState> read = {
           {planarizer_pitch_joint, {}},
           {planarizer_yaw_joint, {}},
           {boom_connector_joint, {}},
           {hip_joint, {}},
           {knee_joint, {}}
       };

      /**
       * @brief Mutex lock for Setting buffer
       */
      std::mutex settings_door;

     /**
      * @brief Bool indicator whether Setting buffer was modified
      */
     bool pid_modified = false;


      // /**
      //  * @brief Bool indicator whether Setting buffer was modified
      //  */
      // using JointSettingModified = bool;
      // std::unordered_map<JointNameIndexing, JointSettingModified> settings_modified = {
      //     {planarizer_pitch_joint, true},
      //     {planarizer_yaw_joint, true},
      //     {boom_connector_joint, true},
      //     {hip_joint, true},
      //     {knee_joint, true}
      // };

      /**
       * @brief Setting Write Buffer
       */
      std::unordered_map<JointNameIndexing, JointSettingsState> settings = {
          {planarizer_pitch_joint, {}},
          {planarizer_yaw_joint, {}},
          {boom_connector_joint, {}},
          {hip_joint, {}},
          {knee_joint, {}}
      };
   } buffers;

};  // end class Monopod definition

}  // namespace monopod_drivers
