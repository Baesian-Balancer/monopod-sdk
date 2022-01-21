#pragma once

#include <real_time_tools/spinner.hpp>
#include <real_time_tools/thread.hpp>
#include <real_time_tools/timer.hpp>
#include <time_series/time_series.hpp>

#include <algorithm>
#include <fstream>
#include <math.h>
#include <mutex>
#include <optional>
#include <tuple>
#include <unordered_map>

#include "monopod_sdk/common_header.hpp"
#include "monopod_sdk/monopod_drivers/leg.hpp"
#include "monopod_sdk/monopod_drivers/planarizer.hpp"

namespace monopod_drivers {
/**
 * @brief Drivers for open sim2real monopod. Interfaces with the monopod TI
 * motors using monopod_drivers::BlmcJointModule. This class creates a real time
 * control thread which reads and writes from a buffer exposed to the public
 * api.
 */
class Monopod {

public:
  struct PID;
  struct JointLimit;

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
   *
   * @param num_joints is the number of joints the robot is running. Supports 2
   * (only leg), 3 (fixed hip_joint and planarizer_yaw_joint),4 (fixed
   * hip_joint), 5 (free).
   * @param hip_home_offset_rad hip offset from found encoder index 0 (rad)
   * @param knee_home_offset_rad knee offset from found encoder index 0 (rad)
   */
  bool initialize(int num_joints = 5, const double &hip_home_offset_rad = 0,
                  const double &knee_home_offset_rad = 0);

  /**
   * @brief is the monopod sdk Initialized?.
   */
  bool initialized();

  /**
   * @brief This method is a helper to start the thread loop. Requires the class
   * to be initialized before the loop can be started.
   */
  void start_loop();

  /**
   * @brief Calibrate the Encoders.
   */
  void calibrate(const double &hip_home_offset_rad = 0,
                 const double &knee_home_offset_rad = 0);

  /**
   * @brief Get model name
   *
   * @return String of model name
   */
  std::string get_model_name() const;

  /**
   * @brief Get a map of 'active' joint strings indexing their enumerator index
   *
   * @return Unordered map of joint name strings as key and index as value
   */
  std::unordered_map<std::string, int> get_joint_names() const;

  /**
   * @brief check if the joint is a controllable joint (has a motor) or only
   * a observation joint (encoder only).
   *
   * @param joint_index name of the joint we want to access
   * @return bool whether joint is controllable
   */
  bool is_joint_controllable(const int joint_index);
  // ======================================
  // setters
  //=======================================

  /**
   * @brief Set the torque target for some joint index. Return a bool whether
   * successful.
   *
   * @param torque_target is the desired torque target for indexed joint
   * @param joint_index name of the joint we want to access
   * @return bool whether setting the value was successfull
   */
  bool set_torque_target(const double &torque_target, const int joint_index);

  /**
   * @brief Set the torque targets for all joint indexes. Return a bool whether
   * successful.
   *
   * @param torque_targets vector of desired torque targets for indexed joints
   * @param joint_indexes names of the joints we want to access
   * @return bool whether setting the value was successfull
   */
  bool set_torque_targets(const std::vector<double> &torque_targets,
                          const std::vector<int> &joint_indexes = {});

  /**
   * Set the PID parameters of the joint.
   *
   * @param pid The desired PID parameters.
   * @return True for success, false otherwise.
   */
  bool set_pid(const int &p, const int &i, const int &d,
               const int &joint_index);

  /**
   * Set the maximum Position of the joint.
   *
   * This limit when reached will kill the robot for safety
   *
   * @param max A double with the maximum position of the joint.
   * @param min A double with the minimum position of the joint.
   * @param joint_index name of the joint we want to access
   * @return True for success, false otherwise.
   */
  bool set_joint_position_limit(const double &max, const double &min,
                                const int &joint_index);

  /**
   * Set the maximum velocity of the joint.
   *
   * This limit when reached will kill the robot for safety
   *
   * @param max A double with the maximum velocity of the joint.
   * @param min A double with the minimum velocity of the joint.
   * @param joint_index name of the joint we want to access
   * @return True for success, false otherwise.
   */
  bool set_joint_velocity_limit(const double &max, const double &min,
                                const int &joint_index);

  /**
   * Set the maximum acceleration of the joint.
   *
   * This limit when reached will kill the robot for safety
   *
   * @param max A double with the maximum acceleration of the joint.
   * @param min A double with the minimum acceleration of the joint.
   * @param joint_index name of the joint we want to access
   * @return True for success, false otherwise.
   */
  bool set_joint_acceleration_limit(const double &max, const double &min,
                                    const int &joint_index);

  /**
   * Set the maximum torque target of the joint.
   *
   * This limit when reached will kill the robot for safety
   *
   * @param max_torque_target A double with the maximum torque of the joint.
   * @param joint_index name of the joint we want to access
   * @return True for success, false otherwise.
   */
  bool set_max_torque_target(const double &max_torque_target,
                             const int &joint_index);

  // ======================================
  // getters
  //=======================================

  /**
   * Get the PID parameters of the joint.
   *
   * If no PID parameters have been set, the default parameters are
   * returned.
   *
   * @return The joint PID parameters.
   */
  std::optional<PID> get_pid(const int &joint_index);

  /**
   * Get the position limits of the joint.
   *
   * @return The position limits of the joint.
   */
  std::optional<JointLimit> get_joint_position_limit(const int &joint_index);

  /**
   * Get the velocity limits of the joint.
   *
   * @return The velocity limits of the joint.
   */
  std::optional<JointLimit> get_joint_velocity_limit(const int &joint_index);

  /**
   * Get the velocity limits of the joint.
   *
   * @return The velocity limits of the joint.
   */
  std::optional<JointLimit>
  get_joint_acceleration_limit(const int &joint_index);

  /**
   * @brief Get the max torque
   *
   * @param joint_index
   * @return std::optional<double> containing the max torque if success
   */
  std::optional<double> get_max_torque_target(const int &joint_index);

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
  std::optional<std::vector<double>>
  get_torque_targets(const std::vector<int> &joint_indexes = {});

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
   * @return std::optional<vector<double>> containing vector of positions if
   * success
   */
  std::optional<std::vector<double>>
  get_positions(const std::vector<int> &joint_indexes = {});

  /**
   * @brief Get the velocity of the joint indexes
   *
   * @param joint_indexes names of the joints we want to access
   * @return std::optional<std::vector<double>> containing vector of velocities
   * if success
   */
  std::optional<std::vector<double>>
  get_velocities(const std::vector<int> &joint_indexes = {});

  /**
   * @brief Get the acceleration of the joint indexes
   *
   * @param joint_indexes names of the joints we want to access
   * @return std::optional<std::vector<double>> containing vector of
   * accelerations if success
   */
  std::optional<std::vector<double>>
  get_accelerations(const std::vector<int> &joint_indexes = {});

private:
  /**
   * @brief this function is just a wrapper around the actual loop function,
   * such that it can be spawned as a posix thread.
   */
  static THREAD_FUNCTION_RETURN_TYPE loop(void *instance_pointer) {
    ((Monopod *)(instance_pointer))->loop();
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

  std::optional<std::vector<double>>
  getJointDataSerialized(const Monopod *monopod,
                         const std::vector<int> &joint_indexes,
                         std::function<double(int)> getJointData) {
    // Take the joint index in lambda. Return the data you want.
    const std::vector<int> &jointSerialization =
        joint_indexes.empty() ? monopod->read_joint_indexing : joint_indexes;

    std::vector<double> data;
    data.reserve(jointSerialization.size());
    for (auto &joint_index : jointSerialization) {
      if (is_initialized && Contains(read_joint_indexing, joint_index)) {
        data.push_back(getJointData(joint_index));
      } else {
        return std::nullopt;
      }
    }
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
      {"planarizer_pitch_joint", planarizer_pitch_joint}};

  /**
   * @brief Structure holding the observed state of a joint
   */
  struct PID {
    PID() = default;
    PID(const double _p, const double _i, const double _d)
        : p(_p), i(_i), d(_d) {}

    double p = 0;
    double i = 0;
    double d = 0;
  };

  /**
   * @brief Structure holding joint limits
   */
  struct JointLimit {
    JointLimit() {
      constexpr double m = std::numeric_limits<double>::lowest();
      constexpr double M = std::numeric_limits<double>::max();

      min = m;
      max = M;
    }

    JointLimit(const double _min, const double _max) : min(_min), max(_max) {}

    double min;
    double max;
  };

  /**
   * @brief Template helper for getting sign
   */
  template <typename T> static int sgn(T val) {
    return (T(0) < val) - (val < T(0));
  }

  /**
   * @brief Template helper checking if vector contains an element.
   */
  template <typename T>
  bool Contains(const std::vector<T> &Vec, const T &Element) const {
    if (std::find(Vec.begin(), Vec.end(), Element) != Vec.end())
      return true;

    return false;
  }

  /**
   * @brief Template helper checking if vector contains an element.
   */
  template <typename T> static bool in_range(T value, T min, T max) {
    return min <= value && value < max;
  }

private:
  /**
   * @brief the realt time thread object.
   */
  real_time_tools::RealTimeThread rt_thread_;

  /**
   * @brief Canbus connection.
   */
  std::shared_ptr<monopod_drivers::CanBus> can_bus_;

  /**
   * @brief Canbus ControlBoards.
   */
  std::shared_ptr<monopod_drivers::CanBusControlBoards> board_;

  /**
   * @brief robot Planarizer interface object
   */
  std::unique_ptr<monopod_drivers::Planarizer> planarizer_;

  /**
   * @brief robot Leg interface object
   */
  std::unique_ptr<monopod_drivers::Leg> leg_;

  /**
   * @brief number joints active.
   */
  long unsigned int num_joints_;

  /**
   * @brief managing the stopping of the loop
   */
  bool stop_loop;

  /**
   * @brief boolen defining if sdk is initialized.
   */
  bool is_initialized;

  /**
   * @brief Write Joint names indexed same as enumerator for actuators.  All
   * valid controlled joints should be defined here.
   */
  std::vector<int> write_joint_indexing;

  /**
   * @brief Read Joint names indexed same as enumerator for encoders. All valid
   * joints should be defined here.
   */
  std::vector<int> read_joint_indexing;

  /**
   * @brief Structure holding the observed state of a joint
   */
  struct JointReadState {
    JointReadState() = default;
    JointReadState(const double _pos, const double _vel, const double _acc)
        : pos(_pos), vel(_vel), acc(_acc) {}
    double pos = 0.0;
    double vel = 0.0;
    double acc = 0.0;
  };

  /**
   * @brief Structure holding the observed state of a joint
   */
  struct JointSettingState {
    JointSettingState() = default;
    JointSettingState(const double _max_torque_target,
                      const JointLimit _position_limit,
                      const JointLimit _velocity_limit,
                      const JointLimit _acceleration_limit)
        : position_limit(_position_limit), velocity_limit(_velocity_limit),
          acceleration_limit(_acceleration_limit),
          max_torque_target(_max_torque_target) {}

    JointLimit position_limit = {};
    JointLimit velocity_limit = {};
    JointLimit acceleration_limit = {};
    double max_torque_target = 0;
  };

  /**
   * @brief Read/Write buffer
   */
  struct Buffers {
    /**
     * @brief Mutex lock for write buffer
     */
    std::mutex write_door;

    /**
     * @brief Write Buffer
     */
    using JointWriteState = double;
    std::unordered_map<JointNameIndexing, JointWriteState> write = {
        {hip_joint, 0.0}, {knee_joint, 0.0}};

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
        {knee_joint, {}}};

    /**
     * @brief Mutex lock for Setting buffer
     */
    std::mutex settings_door;

    /**
     * @brief Setting Write Buffer
     */
    std::unordered_map<JointNameIndexing, JointSettingState> settings = {
        {planarizer_pitch_joint, {}},
        {planarizer_yaw_joint, {}},
        {boom_connector_joint, {}},
        {hip_joint, {}},
        {knee_joint, {}}};

    /**
     * @brief Mutex lock for PID buffer. PID is seperate because it is rarely
     * changed and requires us to send commands to device. this means it is
     */
    std::mutex pid_door;

    /**
     * @brief Bool indicator whether Setting buffer was modified
     */
    bool pid_modified = false;

    /**
     * @brief Setting Write Buffer
     */
    std::unordered_map<JointNameIndexing, PID> pid = {
        {planarizer_pitch_joint, {}},
        {planarizer_yaw_joint, {}},
        {boom_connector_joint, {}},
        {hip_joint, {}},
        {knee_joint, {}}};
  } buffers;

}; // end class Monopod definition

} // namespace monopod_drivers
