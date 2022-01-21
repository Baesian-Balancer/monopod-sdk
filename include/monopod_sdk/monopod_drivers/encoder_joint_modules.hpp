#pragma once

#include <map>
#include <math.h>
#include <memory>
#include <string>

#include <time_series/time_series.hpp>

#include "monopod_sdk/monopod_drivers/devices/device_interface.hpp"
#include "monopod_sdk/monopod_drivers/devices/motor.hpp"
#include "monopod_sdk/monopod_drivers/encoder_joint_module.hpp"

#include "monopod_sdk/common_header.hpp"

namespace monopod_drivers {

/**
 * @brief The Planarizer class is the implementation of the PlanarizerInterface.
 * This is the decalartion and the definition of the class as it is very simple.
 */

class Planarizer {
public:
  /**
   * @brief Construct the PlanarizerInterface object
   */
  Planarizer(std::shared_ptr<ControlBoardsInterface> board) : board_(board) {}

  /**
   * @brief Destroy the PlanarizerInterface object
   */
  ~Planarizer() {}

  /**
   * @brief Initialize canbus connecion, esablish connection to the motors, and
   * set motor constants.
   */
  bool initialize(const int &num_joints) {

    board_->wait_until_ready();
    initialized = true;
    return true;
  }

  // =========================================================================
  //  GETTERS
  // =========================================================================

  /**
   * @brief Get all meassurements of the Planarizer. This includes Position,
   * Velocity, and In the future Acceleration.
   *
   * @return unordered map of std::vector<double> measurements. Indexed with the
   * meassurement type enum.
   */
  ObservationMap get_measurements() const {
    throw_if_not_init();

    // todo : check if valid otherwise enter complete safemode.

    ObservationMap data = {};

    for (const auto &pair : joints_) {
      data[pair.first][position] = pair.second->get_measured_angle();
      data[pair.first][velocity] = pair.second->get_measured_velocity();
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
    throw_if_not_init();

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
   * @brief Calibrate the Planarizer. See blmc_joint_module.hpp for explanation
   * of parameters and logic.
   */
  bool calibrate(const std::vector<double> &home_offset_rad) {
    throw_if_not_init();
    return true;
  }

  /**
   * @brief Set the zero_angles. These are the joint angles between the
   * starting pose and the zero theoretical pose of the urdf.
   *
   * @param zero_angles (rad)
   */
  void set_zero_angles(const std::vector<double> &zero_angles) {
    throw_if_not_init();
    if (zero_angles.size() != num_joints_)
      throw std::runtime_error("need same number of elements as number joints. "
                               "(monopod_drivers::Planarizer)");

    size_t i = 0;
    for (const auto &pair : joints_) {

      pair.second->set_zero_angle(zero_angles[i]);
      i++;
    }
  }

  /**
   * @brief Set the polarities of the joints
   * (see MotorJointModule::set_joint_polarity)
   *
   * @param reverse_polarity
   */
  void set_joint_polarities(std::vector<bool> reverse_polarities) {
    throw_if_not_init();
    if (reverse_polarities.size() != num_joints_)
      throw std::runtime_error("need same number of elements as number joints. "
                               "(monopod_drivers::Planarizer)");

    size_t i = 0;
    for (const auto &pair : joints_) {

      pair.second->set_joint_polarity(reverse_polarities[i]);
      i++;
    }
  }

private:
  /**
   * @brief Check if joint is initialed. otherwise throw error
   */
  void throw_if_not_init() const {
    if (!initialized)
      throw std::runtime_error("Need to initialize the planarizer before use.");
  }
  /**
   * @brief Canbus board for encoders.
   */
  std::shared_ptr<monopod_drivers::ControlBoardsInterface> board_;

  /**
   * @brief number joints active.
   */
  long unsigned int num_joints_;

  /**
   * @brief is Initialized.
   */
  bool initialized = false;

  /**
   * @brief Hip and knee motor modules for the Planarizer
   */
  std::unordered_map<JointNameIndexing,
                     std::shared_ptr<monopod_drivers::EncoderJointModule>>
      joints_;
};

} // namespace monopod_drivers
