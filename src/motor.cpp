/**
 * @file motor.cpp
 * @author Manuel Wuthrich (manuel.wuthrich@gmail.com)
 * @author Maximilien Naveau (maximilien.naveau@gmail.com)
 * @brief
 * @version 0.1
 * @date 2018-11-27
 *
 * @copyright Copyright (c) 2018
 *
 */

#include "monopod_sdk/monopod_drivers/devices/motor.hpp"

namespace monopod_drivers {
Motor::Motor(Motor::Ptr<ControlBoardsInterface> board,
             JointNameIndexing motor_id)
    : Encoder(board, motor_id), board_(board), motor_id_(motor_id) {}

Motor::Ptr<const Motor::ScalarTimeseries>
Motor::get_measurement(const MeasurementIndex &index) const {
  return Encoder::get_measurement(index);
}

Motor::Ptr<const Motor::StatusTimeseries> Motor::get_status() const {
  return Encoder::get_status();
}

Motor::Ptr<const Motor::ScalarTimeseries> Motor::get_current_target() const {
  if (motor_id_ == 0) {
    return board_->get_control(ControlBoardsInterface::current_target_0);
  } else {
    return board_->get_control(ControlBoardsInterface::current_target_1);
  }
}

Motor::Ptr<const Motor::ScalarTimeseries>
Motor::get_sent_current_target() const {
  if (motor_id_ == 0) {
    return board_->get_sent_control(ControlBoardsInterface::current_target_0);
  } else {
    return board_->get_sent_control(ControlBoardsInterface::current_target_1);
  }
}

void Motor::set_current_target(const double &current_target) {
  if (motor_id_ == 0) {
    board_->set_control(current_target,
                        ControlBoardsInterface::current_target_0);
  } else {
    board_->set_control(current_target,
                        ControlBoardsInterface::current_target_1);
  }
}

void Motor::print() const {
  double motor_current = std::nan("");
  double motor_position = std::nan("");
  double motor_velocity = std::nan("");
  double motor_encoder_index = std::nan("");
  double motor_sent_current_target = std::nan("");

  if (get_status()->length() != 0) {

    // TODO: Maybe fix this for the first iteration? right now it wont print on
    // that iteration.
    MotorBoardStatus &motor_board_status =
        dynamic_cast<MotorBoardStatus &>(get_status()->newest_element());

    rt_printf("motor board status: ");
    rt_printf("enabled: %d ", motor_board_status.system_enabled);
    rt_printf("error_code: %d ", motor_board_status.error_code);
    rt_printf("motor status: ");
    if (motor_id_ == hip_joint) {
      rt_printf("enabled: %d ", motor_board_status.motor1_enabled);
      rt_printf("ready: %d ", motor_board_status.motor1_ready);
    } else if (motor_id_ == knee_joint) {
      rt_printf("enabled: %d ", motor_board_status.motor2_enabled);
      rt_printf("ready: %d ", motor_board_status.motor2_ready);
    }
  }

  if (get_measurement(current)->length() != 0) {
    motor_current = get_measurement(current)->newest_element();
  }

  if (get_measurement(position)->length() != 0) {
    motor_position = get_measurement(position)->newest_element();
  }

  if (get_measurement(velocity)->length() != 0) {
    motor_velocity = get_measurement(velocity)->newest_element();
  }

  if (get_measurement(encoder_index)->length() != 0) {
    motor_encoder_index = get_measurement(encoder_index)->newest_element();
  }

  if (get_sent_current_target()->length() != 0) {
    motor_sent_current_target = get_sent_current_target()->newest_element();
  }

  rt_printf("motor measurements: ");
  rt_printf("current: %8f ", motor_current);
  rt_printf("position: %8f ", motor_position);
  rt_printf("velocity: %8f ", motor_velocity);
  rt_printf("encoder index: %8f ", motor_encoder_index);
  rt_printf("target current: %8f ", motor_sent_current_target);
  rt_printf("\n");
}

SafeMotor::SafeMotor(Motor::Ptr<ControlBoardsInterface> board,
                     JointNameIndexing motor_id,
                     const double &max_current_target,
                     const size_t &history_length, const double &max_velocity)
    : Motor(board, motor_id), max_current_target_(max_current_target),
      max_velocity_(max_velocity) {
  current_target_ =
      std::make_shared<ScalarTimeseries>(history_length, 0, false);
}

void SafeMotor::set_current_target(const double &current_target) {
  current_target_->append(current_target);

  // limit current to avoid overheating ----------------------------------
  double safe_current_target = std::min(current_target, max_current_target_);
  safe_current_target = std::max(safe_current_target, -max_current_target_);

  // limit velocity to avoid breaking the robot --------------------------
  if (!std::isnan(max_velocity_) && get_measurement(velocity)->length() > 0 &&
      std::fabs(get_measurement(velocity)->newest_element()) > max_velocity_)
    safe_current_target = 0;
  Motor::set_current_target(safe_current_target);
}

} // namespace monopod_drivers
