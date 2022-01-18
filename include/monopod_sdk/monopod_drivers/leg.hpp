#pragma once

#include <map>
#include <math.h>
#include <memory>
#include <string>

#include <time_series/time_series.hpp>

#include "monopod_sdk/monopod_drivers/devices/device_interface.hpp"
#include "monopod_sdk/monopod_drivers/devices/motor.hpp"
#include "monopod_sdk/monopod_drivers/motor_joint_module.hpp"
#include "monopod_sdk/monopod_drivers/utils/polynome.hpp"

#include "monopod_sdk/common_header.hpp"

namespace monopod_drivers {

/**
 * @brief The leg class is the implementation of the LegInterface. This is
 * the decalartion and the definition of the class as it is very simple.
 */
class Leg {
public:
  /**
   * @brief Enumerate the num_joints For readability
   *
   */
  static constexpr long unsigned int num_joints_ = 2;

  /**
   * @brief Construct the LegInterface object
   */
  Leg(std::shared_ptr<ControlBoardsInterface> board,
      double motor_max_current = 5.0)
      : board_(board), motor_max_current_(motor_max_current) {}

  /**
   * @brief Destroy the LegInterface object
   */
  ~Leg() {}

  /**
   * @brief Initialize canbus connecion, esablish connection to the motors, and
   * set motor constants.
   */
  bool initialize() {

    auto motor_hip_joint = std::make_shared<monopod_drivers::SafeMotor>(
        board_, hip_joint,
        motor_max_current_ * 0.99); // 0.99 means that safe motor will have a
                                    // chance to limit current before

    auto motor_knee_joint = std::make_shared<monopod_drivers::SafeMotor>(
        board_, knee_joint, motor_max_current_ * 0.99);

    joints_[hip_joint] =
        std::make_shared<MotorJointModule>(motor_hip_joint,
                                           0.025, // motor_constants[i],
                                           9.0,   // gear_ratios[i],
                                           0.0,   // zero_angles[i],
                                           false, motor_max_current_);

    joints_[knee_joint] =
        std::make_shared<MotorJointModule>(motor_knee_joint,
                                           0.025, // motor_constants[i],
                                           9.0,   // gear_ratios[i],
                                           0.0,   // zero_angles[i],
                                           false, motor_max_current_);

    // The the control gains in order to perform the calibration
    double kp, kd;
    kp = 3.0;
    kd = 0.05;

    joints_[hip_joint]->set_position_control_gains(kp, kd);
    joints_[knee_joint]->set_position_control_gains(kp, kd);

    // wait until all board are ready and connected
    board_->wait_until_ready();

    return true;
  }

  // =========================================================================
  //  GETTERS
  // =========================================================================

  /**
   * @brief Get all meassurements of the leg. This includes Position, Velocity,
   * Torque, and In the future Acceleration.
   *
   * @return unordered map of LVector measurements. Indexed with the
   * meassurement type enum.
   */
  ObservationMap get_measurements() const {

    ObservationMap data = {};

    for (const auto &pair : joints_) {
      data[pair.first][position] = pair.second->get_measured_angle();
      data[pair.first][velocity] = pair.second->get_measured_velocity();
      data[pair.first][torque] = pair.second->get_measured_torque();
    }

    return data;
  }

  /**
   * @brief Get the zero_angles. These are the joint angles between the
   * starting pose and the zero theoretical pose of the urdf.
   *
   * @return std::vector<double> (rad)
   */
  std::vector<double> get_zero_angles() const {

    std::vector<double> positions;
    positions.reserve(num_joints_);

    for (const auto &pair : joints_) {
      positions.push_back(pair.second->get_zero_angle());
    }
    return positions;
  }

  // =========================================================================
  //  SETTERS
  // =========================================================================

  /**
   * @brief Set the torque targets for each joint.
   *
   * @param torque_targets are the torque to achieve on the motor card. the data
   * format is {hip, knee}
   * @param joint_index is the motor to control.
   */
  void set_target_torques(const std::vector<double> &torque_targets) {
    if (torque_targets.size() != num_joints_)
      throw std::runtime_error(
          "need same number of elements as number joints.");

    joints_[hip_joint]->set_torque(torque_targets[0]);
    joints_[knee_joint]->set_torque(torque_targets[1]);
  }

  /**
   * @brief Send the set torque
   */
  void send_target_torques() {
    joints_[hip_joint]->send_torque();
    joints_[knee_joint]->send_torque();
  }

  /**
   * @brief Set the zero_angles. These are the joint angles between the
   * starting pose and the zero theoretical pose of the urdf.
   *
   * @param zero_angles (rad)
   */
  void set_zero_angles(const std::vector<double> &zero_angles) {
    if (zero_angles.size() != num_joints_)
      throw std::runtime_error(
          "need same number of elements as number joints.");

    size_t i = 0;
    for (const auto &pair : joints_) {

      pair.second->set_zero_angle(zero_angles[i]);
      i++;
    }
  }

  /**
   * @brief Set the polarities of the joints
   * (see BlmcJointModule::set_joint_polarity)
   *
   * @param reverse_polarity
   */
  void set_joint_polarities(std::vector<bool> reverse_polarities) {
    if (reverse_polarities.size() != num_joints_)
      throw std::runtime_error(
          "need same number of elements as number joints.");

    size_t i = 0;
    for (const auto &pair : joints_) {

      pair.second->set_joint_polarity(reverse_polarities[i]);
      i++;
    }
  }

  /**
   * @brief Calibrate the leg. See motor_joint_module.hpp for explanation of
   * parameters and logic.
   */
  bool calibrate(const double &hip_home_offset_rad,
                 const double &knee_home_offset_rad) {
    double search_distance_limit_rad = 2 * M_PI;
    LVector home_offset_rad = {hip_home_offset_rad, knee_home_offset_rad};
    execute_homing(search_distance_limit_rad, home_offset_rad);
    LVector zero_pose = LVector::Zero();
    go_to(zero_pose);
    return true;
  }

private:
  /**
   * @brief Canbus ControlBoards.
   */
  std::shared_ptr<monopod_drivers::ControlBoardsInterface> board_;

  /**
   * @brief Hip and knee joint modules for the leg
   */
  std::unordered_map<JointNameIndexing,
                     std::shared_ptr<monopod_drivers::MotorJointModule>>
      joints_;

  /**
   * @brief Max allowable current for the joint.
   */
  double motor_max_current_;

private:
  /**
   * @brief Perform homing for all joints.
   *
   * If one of the joints fails, the complete homing fails.  Otherwise it
   * loops until all joints finished.
   * If a joint is finished while others are still running, it is held at the
   * home position.
   *
   * @param search_distance_limit_rad  Maximum distance the motor moves while
   *     searching for the encoder index.  Unit: radian.
   * @param hip_home_offset_rad  Offset from home position to zero position.
   *     Unit: radian.
   * @param knee_home_offset_rad  Offset from home position to zero position.
   *     Unit: radian.
   * @param profile_step_size_rad  Distance by which the target position of
   * the position profile is changed in each step.  Set to a negative value to
   *     search for the next encoder index in negative direction.  Unit:
   *     radian.

   * @return Final status of the homing procedure (either SUCCESS if all
   *     joints succeeded or the return code of the first joint that failed).
   */
  HomingReturnCode execute_homing(double search_distance_limit_rad,
                                  LVector home_offset_rad,
                                  double profile_step_size_rad = 0.001) {
    // Initialise homing for all joints

    joints_[hip_joint]->init_homing((int)hip_joint, search_distance_limit_rad,
                                    home_offset_rad[0], profile_step_size_rad);

    joints_[knee_joint]->init_homing((int)knee_joint, search_distance_limit_rad,
                                     home_offset_rad[1], profile_step_size_rad);

    // run homing for all joints until all of them are done
    real_time_tools::Spinner spinner;
    spinner.set_period(0.001); // TODO magic number
    HomingReturnCode homing_status;
    do {
      bool all_succeeded = true;
      homing_status = HomingReturnCode::RUNNING;

      for (const auto &joint_pair : joints_) {
        HomingReturnCode joint_result = joint_pair.second->update_homing();

        all_succeeded &= (joint_result == HomingReturnCode::SUCCEEDED);

        if (joint_result == HomingReturnCode::NOT_INITIALIZED ||
            joint_result == HomingReturnCode::FAILED) {
          homing_status = joint_result;
          // abort homing
          break;
        }

        if (homing_status == HomingReturnCode::RUNNING) {
          joint_pair.second->send_torque();
        }
      }

      if (all_succeeded) {
        homing_status = HomingReturnCode::SUCCEEDED;
      }

      spinner.spin();
    } while (homing_status == HomingReturnCode::RUNNING);

    return homing_status;
  }

  /**
   * @brief Allow the robot to go to a desired pose. Once the control done
   * 0 torques is sent.
   *
   * @param angle_to_reach_rad (rad) in the order [hip_joint, knee_joint]
   * @param average_speed_rad_per_sec (rad/sec)
   * @return GoToReturnCode
   */
  GoToReturnCode go_to(LVector angle_to_reach_rad,
                       double average_speed_rad_per_sec = 1.0) {
    // Compute a min jerk trajectory
    LVector initial_joint_positions = {
        joints_[hip_joint]->get_measured_angle(),
        joints_[knee_joint]->get_measured_angle()};

    double final_time = (angle_to_reach_rad - initial_joint_positions)
                            .array()
                            .abs()
                            .maxCoeff() /
                        average_speed_rad_per_sec;

    std::array<TimePolynome<5>, num_joints_> min_jerk_trajs;
    for (unsigned i = 0; i < num_joints_; i++) {
      min_jerk_trajs[i].set_parameters(final_time, initial_joint_positions[i],
                                       0.0 /*initial speed*/,
                                       angle_to_reach_rad[i]);
    }

    // run got_to for all joints
    real_time_tools::Spinner spinner;
    double sampling_period = 0.001; // TODO magic number
    spinner.set_period(sampling_period);
    GoToReturnCode go_to_status;
    double current_time = 0.0;

    double desired_pose;
    double desired_torque;
    do {
      // TODO: add a security if error gets too big
      desired_pose = min_jerk_trajs[0].compute(current_time);
      desired_torque =
          joints_[hip_joint]->execute_position_controller(desired_pose);
      joints_[hip_joint]->set_torque(desired_torque);

      desired_pose = min_jerk_trajs[1].compute(current_time);
      desired_torque =
          joints_[knee_joint]->execute_position_controller(desired_pose);
      joints_[knee_joint]->set_torque(desired_torque);

      joints_[hip_joint]->send_torque();
      joints_[knee_joint]->send_torque();

      go_to_status = GoToReturnCode::RUNNING;

      current_time += sampling_period;
      spinner.spin();

    } while (current_time < (final_time + sampling_period));

    // Stop all motors (0 torques) after the destination achieved
    for (const auto &joint_pair : joints_) {
      joint_pair.second->set_torque(0.0);
      joint_pair.second->send_torque();
    }

    LVector final_pos = {joints_[hip_joint]->get_measured_angle(),
                         joints_[knee_joint]->get_measured_angle()};
    if ((angle_to_reach_rad - final_pos).isMuchSmallerThan(1.0, 1e-3)) {
      go_to_status = GoToReturnCode::SUCCEEDED;
    } else {
      go_to_status = GoToReturnCode::FAILED;
    }
    return go_to_status;
  }
};

} // namespace monopod_drivers
