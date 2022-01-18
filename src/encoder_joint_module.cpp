#include "monopod_sdk/monopod_drivers/encoder_joint_module.hpp"
#include "real_time_tools/iostream.hpp"
#include "real_time_tools/spinner.hpp"
#include <cmath>

namespace monopod_drivers {

EncoderJointModule::EncoderJointModule(
    std::shared_ptr<monopod_drivers::EncoderInterface> encoder,
    const double &gear_ratio, const double &zero_angle,
    const bool &reverse_polarity) {
  encoder_ = encoder;
  gear_ratio_ = gear_ratio;
  set_zero_angle(zero_angle);
  polarity_ = reverse_polarity ? -1.0 : 1.0;
}

void EncoderJointModule::set_zero_angle(const double &zero_angle) {
  zero_angle_ = zero_angle;
}

void EncoderJointModule::set_joint_polarity(const bool &reverse_polarity) {
  polarity_ = reverse_polarity ? -1.0 : 1.0;
}

double EncoderJointModule::get_measured_angle() const {
  return get_encoder_measurement(MeasurementIndex::position) / gear_ratio_ -
         zero_angle_;
}

double EncoderJointModule::get_measured_velocity() const {
  return get_encoder_measurement(MeasurementIndex::velocity) / gear_ratio_;
}

double EncoderJointModule::get_measured_index_angle() const {
  return get_encoder_measurement(MeasurementIndex::encoder_index) / gear_ratio_;
}

double EncoderJointModule::get_zero_angle() const { return zero_angle_; }

double EncoderJointModule::get_encoder_measurement(
    const MeasurementIndex &measurement_id) const {
  auto measurement_history = encoder_->get_measurement(measurement_id);

  if (measurement_history->length() == 0) {
    // rt_printf("get_encoder_measurement returns NaN\n");
    return std::numeric_limits<double>::quiet_NaN();
  }
  return polarity_ * measurement_history->newest_element();
}

long int EncoderJointModule::get_encoder_measurement_index(
    const MeasurementIndex &measurement_id) const {
  auto measurement_history = encoder_->get_measurement(measurement_id);

  if (measurement_history->length() == 0) {
    // rt_printf("get_encoder_measurement_index returns NaN\n");
    return -1;
  }
  return measurement_history->newest_timeindex();
}

} // namespace monopod_drivers
