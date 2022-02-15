#include "monopod_sdk/monopod_drivers/motor_joint_module.hpp"
#include "real_time_tools/iostream.hpp"
#include "real_time_tools/spinner.hpp"
#include <cmath>

namespace monopod_drivers {

MotorJointModule::MotorJointModule(
    JointNamesIndex joint_id,
    std::shared_ptr<monopod_drivers::MotorInterface> motor,
    const double &motor_constant, const double &gear_ratio,
    const double &zero_angle, const bool &reverse_polarity)
    : EncoderJointModule(joint_id, motor, gear_ratio, zero_angle,
                         reverse_polarity),
      motor_(motor), motor_constant_(motor_constant) {

  set_zero_angle(zero_angle);

  position_control_gain_p_ = 0;
  position_control_gain_d_ = 0;
}

void MotorJointModule::set_torque(const double &desired_torque) {

  double desired_current = joint_torque_to_motor_current(desired_torque);
  rt_printf("Desired Torque: %.3f, Desired Current: %.3f", desired_torque,
            desired_current);
  // limit current to avoid overheating etc
  // ----------------------------------
  desired_current = std::min(desired_current, max_current_);
  desired_current = std::max(desired_current, -max_current_);

  // Make sure your max isnt above Global max --------------------------------
  if (std::fabs(desired_current) > MAX_CURRENT) {
    std::cerr << "something went wrong, it should never happen"
                 " that desired_current > "
              << MAX_CURRENT << ". desired_current: " << desired_current
              << std::endl;
    exit(-1);
  }

  motor_->set_current_target(polarity_ * desired_current);
}

void MotorJointModule::send_torque() { motor_->send_if_input_changed(); }

double MotorJointModule::get_max_torque() const {
  return motor_current_to_joint_torque(max_current_);
}

void MotorJointModule::set_max_torque(const double &max_torque) {
  max_current_ = motor_current_to_joint_torque(max_torque);
}

double MotorJointModule::get_sent_torque() const {
  auto measurement_history = motor_->get_sent_current_target();

  if (measurement_history->length() == 0) {
    return std::numeric_limits<double>::quiet_NaN();
  }

  return motor_current_to_joint_torque(measurement_history->newest_element());
}

double MotorJointModule::get_measured_torque() const {
  return motor_current_to_joint_torque(
      get_joint_measurement(Measurements::current));
}

double MotorJointModule::joint_torque_to_motor_current(double torque) const {
  return torque / gear_ratio_ / motor_constant_;
}

double MotorJointModule::motor_current_to_joint_torque(double current) const {
  return current * gear_ratio_ * motor_constant_;
}

void MotorJointModule::set_position_control_gains(double kp, double kd) {
  position_control_gain_p_ = kp;
  position_control_gain_d_ = kd;
}

double MotorJointModule::execute_position_controller(
    double target_position_rad) const {
  double diff = target_position_rad - get_measured_angle();

  // simple PD control
  double desired_torque = position_control_gain_p_ * diff -
                          position_control_gain_d_ * get_measured_velocity();

  return desired_torque;
}

void MotorJointModule::homing_at_current_position(double home_offset_rad) {
  // reset the internal zero angle.
  set_zero_angle(0.0);

  // set the zero angle
  set_zero_angle(get_measured_angle() + home_offset_rad);

  homing_state_.status = HomingReturnCode::SUCCEEDED;
}

void MotorJointModule::init_homing(double search_distance_limit_rad,
                                   double home_offset_rad,
                                   double profile_step_size_rad) {
  // reset the internal zero angle.
  set_zero_angle(0.0);

  // TODO: would be nice if the joint instance had a `name` or `id` and class
  // level instead of storing it here (to make more useful debug prints).
  homing_state_.joint_id = joint_id_;

  homing_state_.search_distance_limit_rad = search_distance_limit_rad;
  homing_state_.home_offset_rad = home_offset_rad;
  homing_state_.profile_step_size_rad = profile_step_size_rad;
  homing_state_.last_encoder_index_time_index =
      get_joint_measurement_index(Measurements::encoder_index);
  homing_state_.target_position_rad = get_measured_angle();

  rt_printf("target pos init... %.3f \n", homing_state_.target_position_rad);

  homing_state_.step_count = 0;
  homing_state_.start_position = get_measured_angle();

  homing_state_.status = HomingReturnCode::RUNNING;
}

HomingReturnCode MotorJointModule::update_homing() {
  switch (homing_state_.status) {
  case HomingReturnCode::NOT_INITIALIZED:
    set_torque(0.0);
    send_torque();
    rt_printf("[%d] Homing is not initialized.  Abort.\n",
              homing_state_.joint_id);
    break;

  case HomingReturnCode::FAILED:
    // when failed, send zero-torque commands
    set_torque(0.0);
    break;

  case HomingReturnCode::SUCCEEDED: {
    // when succeeded, keep the motor at the home position
    double desired_torque =
        execute_position_controller(homing_state_.target_position_rad);

    set_torque(desired_torque);
    break;
  }

  case HomingReturnCode::RUNNING: {
    // number of steps after which the distance limit is reached
    const uint32_t max_step_count =
        std::abs(homing_state_.search_distance_limit_rad /
                 homing_state_.profile_step_size_rad);

    // abort if distance limit is reached
    if (homing_state_.step_count >= max_step_count) {
      set_torque(0.0);
      homing_state_.status = HomingReturnCode::FAILED;

      rt_printf("MotorJointModule::update_homing(): "
                "ERROR: Failed to find index with joint [%d].\n",
                homing_state_.joint_id);
      break;
    }

    // -- EXECUTE ONE STEP

    homing_state_.step_count++;
    homing_state_.target_position_rad += homing_state_.profile_step_size_rad;

#ifdef VERBOSE
    const double current_position = get_measured_angle();
    if (homing_state_.step_count % 100 == 0) {
      rt_printf("[%d] cur: %f,\t des: %f\n", homing_state_.joint_id,
                current_position, homing_state_.target_position_rad);
    }
#endif

    // FIXME: add a safety check to stop if following error gets too
    // big.

    const double desired_torque =
        execute_position_controller(homing_state_.target_position_rad);
    set_torque(desired_torque);

    // Check if new encoder index was observed
    const long int actual_index_time =
        get_joint_measurement_index(Measurements::encoder_index);
    if (actual_index_time > homing_state_.last_encoder_index_time_index) {
      // -- FINISHED
      const double index_angle = get_measured_index_angle();
      rt_printf("joint [%d] found encoder index at position [%f]. \n",
                homing_state_.joint_id, index_angle);

      // Store the end position of the homing so it can be used to
      // determine the travelled distance.
      homing_state_.end_position = index_angle;

      // set the zero angle
      set_zero_angle(index_angle + homing_state_.home_offset_rad);

      // adjust target_position according to the new zero
      homing_state_.target_position_rad -= zero_angle_;

#ifdef VERBOSE
      rt_printf("[%d] Zero angle is=%f\n", homing_state_.joint_id, zero_angle_);
      rt_printf("[%d] Index angle is=%f\n", homing_state_.joint_id,
                index_angle);
#endif

      homing_state_.status = HomingReturnCode::SUCCEEDED;
    }

    break;
  }
  }

  return homing_state_.status;
}

double MotorJointModule::get_distance_travelled_during_homing() const {
  if (homing_state_.status != HomingReturnCode::SUCCEEDED) {
    throw std::runtime_error(
        "Homing status needs to be SUCCEEDED to determine travelled "
        "distance.");
  }

  return homing_state_.end_position - homing_state_.start_position;
}

} // namespace monopod_drivers
