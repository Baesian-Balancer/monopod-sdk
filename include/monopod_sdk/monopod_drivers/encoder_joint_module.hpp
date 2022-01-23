#pragma once

#include <array>
#include <iostream>
#include <math.h>
#include <stdexcept>

#include <Eigen/Eigen>

#include "monopod_sdk/common_header.hpp"
#include "monopod_sdk/monopod_drivers/devices/encoder.hpp"

namespace monopod_drivers {

/**
 * @brief The EncoderJointModule class is containing the joint information. It
 * is here to help converting the data from the encoder side to the joint side.
 * It also allows the calibration of the joint position during initialization.
 */
class EncoderJointModule {
public:
  /**
   * @brief Construct a new EncoderJointModule object
   *
   * @param encoder is the C++ object allowing us to receive
   * sensor data.
   * @param gear_ratio is the gear ratio between the encoder and the joint.
   * @param zero_angle is the angle between the closest positive encoder index
   * and the zero configuration.
   * @param reverse_polarity
   */
  EncoderJointModule(JointNameIndexing joint_id,
                     std::shared_ptr<monopod_drivers::EncoderInterface> encoder,
                     const double &gear_ratio, const double &zero_angle,
                     const bool &reverse_polarity = false);

  /**
   * @brief Is the robot in a valid state?
   */
  virtual void valid();

  /**
   * @brief If the joint module is not valid (safemode after limit reached) the
   * joint will be reset into a valid state. This means the joint must be set
   * back into the valid state first otherwise it will trigger the limits again.
   */
  virtual void reset();

  /**
   * @brief Set the zero_angle. The zero_angle is the angle between the
   * closest positive encoder index and the zero configuration.
   *
   * @param zero_angle (rad)
   */
  virtual void set_zero_angle(const double &zero_angle);

  /**
   * @brief Define if the encoder should turn clock-wize or counter clock-wize.
   *
   * @param reverse_polarity true:reverse rotation axis, false:do nothing.
   */
  virtual void set_joint_polarity(const bool &reverse_polarity);

  /**
   * @brief Get the measured angle of the joint.
   *
   * @return double (rad).
   */
  virtual double get_measured_angle() const;

  /**
   * @brief Get the measured velocity of the joint. This data is computed on
   * board of the control card.
   *
   * @return double (rad/s).
   */
  virtual double get_measured_velocity() const;

  /**
   * @brief Get the measured acceleration of the joint. This data is computed on
   * board of the control card.
   *
   * @return double (rad/s^2).
   */
  virtual double get_measured_acceleration() const;

  /**
   * @brief Get the measured index angle. There is one index per encoder
   * rotation so there are gear_ratio indexes per joint rotation.
   *
   * @return double (rad).
   */
  virtual double get_measured_index_angle() const;

  /**
   * @brief Get the zero_angle_. These are the angle between the starting pose
   * and the theoretical zero pose.
   *
   * @return double (rad).
   */
  virtual double get_zero_angle() const;

  /**
   * @brief Set the limit of the provided meassurement index.
   *
   * @param index of the position type to set limit of
   * @param limit is a struct holding the limit for the specified meassurement.
   */
  virtual void set_limit(const MeasurementIndex &index,
                         const JointLimit &limit);

  /**
   * @brief Get the limit of the provided meassurement index.
   *
   * @param index of the position type to set limit of
   */
  virtual JointLimit get_limit(const MeasurementIndex &index) const;

protected:
  /**
   * @brief Get encoder measurements and check if there are data or not.
   *
   * @param measurement_id is the id of the measurement you want to get.
   * check: monopod_drivers::EncoderInterface::MeasurementIndex
   * @return double the measurement.
   */
  virtual double
  get_joint_measurement(const MeasurementIndex &measurement_id) const;

  /**
   * @brief Get the last encoder measurement index for a specific data. If there
   * was no data yet, return NaN
   *
   * @param measurement_id is the id of the measurement you want to get.
   * check: monopod_drivers::EncoderInterface::MeasurementIndex
   * @return double the measurement.
   */
  virtual long int
  get_joint_measurement_index(const MeasurementIndex &measurement_id) const;

  /**
   * @brief This is the joint ID used when initializing the joint.
   */
  JointNameIndexing joint_id_;

  /**
   * @brief This is the pointer to the encoder interface.
   */
  std::shared_ptr<monopod_drivers::EncoderInterface> encoder_;

  /**
   * @brief This is the map of the limits for each meassurement.
   */
  std::unordered_map<MeasurementIndex, JointLimit> limits_ = {
      {position, {}}, {velocity, {}}, {acceleration, {}}};

  /**
   * @brief This correspond to the reduction (\f$ \beta \f$) between the encoder
   * rotation and the joint. \f$ \theta_{joint} = \theta_{encoder} / \beta \f$
   */
  double gear_ratio_;

  /**
   * @brief This is the distance between the closest positive index and the
   * zero configuration.
   */
  double zero_angle_;

  /**
   * @brief This change the encoder rotation direction.
   */
  double polarity_;
};

} // namespace monopod_drivers
