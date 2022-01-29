#include "monopod_sdk/monopod_drivers/devices/encoder.hpp"

namespace monopod_drivers {

Encoder::Encoder(Ptr<monopod_drivers::ControlBoardsInterface> board,
                 monopod_drivers::JointNamesIndex encoder_id)
    : board_(board), encoder_id_(encoder_id) {
  set_board_active();
}

Ptr<const ScalarTimeseries>
Encoder::get_measurement(const Measurements &index) const {
  switch (encoder_id_) {
  case hip_joint:
    switch (index) {
    case position:
      return board_->get_measurement(ControlBoardsInterface::position_0);
    case velocity:
      return board_->get_measurement(ControlBoardsInterface::velocity_0);
    case acceleration:
      throw std::invalid_argument("acceleration not supported yet.");
    case encoder_index:
      return board_->get_measurement(ControlBoardsInterface::encoder_index_0);
    default:
      break;
    }
    break;
  case knee_joint:
    switch (index) {
    case position:
      return board_->get_measurement(ControlBoardsInterface::position_1);
    case velocity:
      return board_->get_measurement(ControlBoardsInterface::velocity_1);
    case acceleration:
      throw std::invalid_argument("acceleration not supported yet.");
    case encoder_index:
      return board_->get_measurement(ControlBoardsInterface::encoder_index_1);
    default:
      break;
    }
    break;
  case planarizer_pitch_joint:
    switch (index) {
    case position:
      return board_->get_measurement(ControlBoardsInterface::position_2);
    case velocity:
      return board_->get_measurement(ControlBoardsInterface::velocity_2);
    case acceleration:
      throw std::invalid_argument("acceleration not supported yet.");
    case encoder_index:
      return board_->get_measurement(ControlBoardsInterface::encoder_index_2);
    default:
      break;
    }
    break;
  case planarizer_yaw_joint:
    switch (index) {
    case position:
      return board_->get_measurement(ControlBoardsInterface::position_3);
    case velocity:
      return board_->get_measurement(ControlBoardsInterface::velocity_3);
    case acceleration:
      throw std::invalid_argument("acceleration not supported yet.");
    case encoder_index:
      return board_->get_measurement(ControlBoardsInterface::encoder_index_3);
    default:
      break;
    }
    break;
  case boom_connector_joint:
    switch (index) {
    case position:
      return board_->get_measurement(ControlBoardsInterface::position_4);
    case velocity:
      return board_->get_measurement(ControlBoardsInterface::velocity_4);
    case acceleration:
      throw std::invalid_argument("acceleration not supported yet.");
    case encoder_index:
      return board_->get_measurement(ControlBoardsInterface::encoder_index_4);
    default:
      break;
    }
    break;
  }

  throw std::invalid_argument("index needs to match one of the measurements");
}

Ptr<const Encoder::StatusTimeseries> Encoder::get_status() const {
  switch (encoder_id_) {
  case hip_joint:
  case knee_joint:
    return board_->get_status(ControlBoardsInterface::motor_board);
  case planarizer_pitch_joint:
  case planarizer_yaw_joint:
    return board_->get_status(ControlBoardsInterface::encoder_board1);
  case boom_connector_joint:
    return board_->get_status(ControlBoardsInterface::encoder_board2);
  }
  throw std::invalid_argument(
      "index needs to match one of the boards in control board interface");
}

void Encoder::set_board_active() const {
  switch (encoder_id_) {
  case hip_joint:
  case knee_joint:
    board_->set_active_board(ControlBoardsInterface::motor_board);
    return;
  case planarizer_pitch_joint:
  case planarizer_yaw_joint:
    board_->set_active_board(ControlBoardsInterface::encoder_board1);
    return;
  case boom_connector_joint:
    board_->set_active_board(ControlBoardsInterface::encoder_board2);
    return;
  }
  throw std::invalid_argument(
      "encoder_id needs to match one of the joints in "
      "the JointNamesIndex enum. The provided value was, " +
      std::to_string(encoder_id_));
}

void Encoder::print() const {
  BoardStatus encoder_board_status;
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
  rt_printf("joint index %d; ", encoder_id_);
  rt_printf("error_code: %d ", encoder_board_status.get_error_code());

  rt_printf("Encoder status: ");
  rt_printf("joint index %d; ", encoder_id_);
  rt_printf("Encoder measurements: ");
  rt_printf("raw position: %8f ", encoder_position);
  rt_printf("raw velocity: %8f ", encoder_velocity);
  rt_printf("raw encoder index: %8f ", encoder_encoder_index);
  rt_printf("\n");
}

} // namespace monopod_drivers
