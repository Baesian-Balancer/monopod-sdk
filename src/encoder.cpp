/**
 * @file Encoder.cpp
 * @author Manuel Wuthrich (manuel.wuthrich@gmail.com)
 * @author Maximilien Naveau (maximilien.naveau@gmail.com)
 * @brief
 * @version 0.1
 * @date 2018-11-27
 *
 * @copyright Copyright (c) 2018
 *
 */

#include "monopod_sdk/monopod_drivers/devices/encoder.hpp"

namespace monopod_drivers {

Encoder::Encoder(Encoder::Ptr<monopod_drivers::ControlBoardsInterface> board,
                 monopod_drivers::JointNameIndexing encoder_id)
    : board_(board), encoder_id_(encoder_id) {}

Encoder::Ptr<const Encoder::ScalarTimeseries>
Encoder::get_measurement(const MeasurementIndex &index) const {
  switch (encoder_id_) {
  case hip_joint:
    switch (index) {
    case current:
      return board_->get_measurement(ControlBoardsInterface::current_0);
    case position:
      return board_->get_measurement(ControlBoardsInterface::position_0);
    case velocity:
      return board_->get_measurement(ControlBoardsInterface::velocity_0);
    case acceleration:
      throw std::invalid_argument("acceleration not supported yet.");
    case encoder_index:
      return board_->get_measurement(ControlBoardsInterface::encoder_index_0);
    }
    break;
  case knee_joint:
    switch (index) {
    case current:
      return board_->get_measurement(ControlBoardsInterface::current_1);
    case position:
      return board_->get_measurement(ControlBoardsInterface::position_1);
    case velocity:
      return board_->get_measurement(ControlBoardsInterface::velocity_1);
    case acceleration:
      throw std::invalid_argument("acceleration not supported yet.");
    case encoder_index:
      return board_->get_measurement(ControlBoardsInterface::encoder_index_1);
    }
    break;
  case boom_connector_joint:
  case planarizer_yaw_joint:
  case planarizer_pitch_joint:
    break;
  }

  throw std::invalid_argument("index needs to match one of the measurements");
}
Encoder::Ptr<const Encoder::StatusTimeseries> Encoder::get_status() const {
  switch (encoder_id_) {
  case hip_joint:
  case knee_joint:
    return board_->get_status(ControlBoardsInterface::motor_board);
  case planarizer_pitch_joint:
  case planarizer_yaw_joint:
    return board_->get_status(ControlBoardsInterface::encoder_board1);
  case boom_connector_joint:
    return board_->get_status(ControlBoardsInterface::encoder_board2);
    break;
  }
  throw std::invalid_argument(
      "index needs to match one of the boards in control board interface");
}

void Encoder::print() const {
  BoardStatusInterface encoder_board_status;
  double encoder_current = std::nan("");
  double encoder_position = std::nan("");
  double encoder_velocity = std::nan("");
  double encoder_encoder_index = std::nan("");

  if (get_status()->length() != 0) {
    encoder_board_status = get_status()->newest_element();
  }

  if (get_measurement(position)->length() != 0) {
    encoder_position = get_measurement(position)->newest_element();
  }

  if (get_measurement(velocity)->length() != 0) {
    encoder_velocity = get_measurement(velocity)->newest_element();
  }

  if (get_measurement(encoder_index)->length() != 0) {
    encoder_encoder_index = get_measurement(encoder_index)->newest_element();
  }

  rt_printf("Encoder board status: ");
  rt_printf("error_code: %d ", encoder_board_status.get_error_code());

  rt_printf("Encoder status: ");
  rt_printf("Encoder measurements: ");
  rt_printf("position: %8f ", encoder_position);
  rt_printf("velocity: %8f ", encoder_velocity);
  rt_printf("encoder index: %8f ", encoder_encoder_index);
  rt_printf("\n");
}

} // namespace monopod_drivers
