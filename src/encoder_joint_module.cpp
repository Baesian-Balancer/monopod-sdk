#include "monopod_sdk/monopod_drivers/encoder_joint_module.hpp"
#include "real_time_tools/iostream.hpp"
#include "real_time_tools/spinner.hpp"
#include <cmath>

namespace monopod_drivers {

EncoderJointModule::EncoderJointModule(
    JointNamesIndex joint_id,
    std::shared_ptr<monopod_drivers::EncoderInterface> encoder,
    const double &gear_ratio, const double &zero_angle,
    const bool &reverse_polarity)
    : joint_id_(joint_id), encoder_(encoder), gear_ratio_(gear_ratio),
      polarity_(reverse_polarity ? -1.0 : 1.0) {
  set_zero_angle(zero_angle);
}

// Todo: implement these two methods
void EncoderJointModule::valid() {}
void EncoderJointModule::reset() {}

void EncoderJointModule::set_zero_angle(const double &zero_angle) {
  zero_angle_ = zero_angle;
}

void EncoderJointModule::set_joint_polarity(const bool &reverse_polarity) {
  polarity_ = reverse_polarity ? -1.0 : 1.0;
}

double EncoderJointModule::get_measured_angle() const {
  return get_joint_measurement(Measurements::position) / gear_ratio_ -
         zero_angle_;
}

double EncoderJointModule::get_measured_velocity() const {
  return get_joint_measurement(Measurements::velocity) / gear_ratio_;
}

double EncoderJointModule::get_measured_acceleration() const {
  return get_joint_measurement(Measurements::acceleration) / gear_ratio_;
}

double EncoderJointModule::get_measured_index_angle() const {
  return get_joint_measurement(Measurements::encoder_index) / gear_ratio_;
}

double EncoderJointModule::get_zero_angle() const { return zero_angle_; }

double EncoderJointModule::get_joint_measurement(
    const Measurements &measurement_id) const {
  auto measurement_history = encoder_->get_measurement(measurement_id);

  if (measurement_history->length() == 0) {
    return std::numeric_limits<double>::quiet_NaN();
  }

  return polarity_ * measurement_history->newest_element();
}

long int EncoderJointModule::get_joint_measurement_index(
    const Measurements &measurement_id) const {
  auto measurement_history = encoder_->get_measurement(measurement_id);

  if (measurement_history->length() == 0) {
    return -1;
  }

  return measurement_history->newest_timeindex();
}

void EncoderJointModule::set_limit(const Measurements &index,
                                   const JointLimit &limit) {
  limits_[index] = limit;
}

JointLimit EncoderJointModule::get_limit(const Measurements &index) const {
  return limits_.at(index);
}
} // namespace monopod_drivers
