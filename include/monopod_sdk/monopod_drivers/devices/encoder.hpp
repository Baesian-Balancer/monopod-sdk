/**
 * @file motor.hpp
 * @license License BSD-3-Clause
 * @copyright Copyright (c) 2019-2020, New York University and Max Planck
 * Gesellschaft.
 * @date 2019-07-11
 */

#pragma once

#include <memory>
#include <string>

#include <real_time_tools/timer.hpp>
#include <time_series/time_series.hpp>

#include "monopod_sdk/common_header.hpp"
#include "monopod_sdk/monopod_drivers/devices/boards.hpp"
#include "monopod_sdk/monopod_drivers/devices/device_interface.hpp"

namespace monopod_drivers {
/**
 * @brief This class declares an interface to the motor. It allows the user to
 * access the sensors data as well as sending controls. The only control
 * supported for now is the current.
 */
class EncoderInterface : public DeviceInterface {
public:
  /**
   * @brief This is a useful alias.
   */
  typedef time_series::TimeSeries<double> ScalarTimeseries;

  /**
   * @brief A useful shortcut
   */
  typedef time_series::TimeSeries<BoardStatusInterface> StatusTimeseries;

  /**
   * @brief A useful shortcut
   */
  typedef ControlBoardsInterface::BoardIndex BoardIndex;

  /**
   * @brief This a useful alias for the shared Pointer creation.
   *
   * @tparam Type is the Class to crate the pointer from.
   */
  template <typename Type> using Ptr = std::shared_ptr<Type>;

  /**
   * @brief Destroy the EncoderInterface object
   */
  virtual ~EncoderInterface() {}

  /**
   * Getters
   */

  /**
   * @brief Get the measurements.
   *
   * @param index
   * @return Ptr<const ScalarTimeseries> the pointer to the desired
   * measurement history.
   */
  virtual Ptr<const ScalarTimeseries>
  get_measurement(const MeasurementIndex &index) const = 0;

  /**
   * @brief Get the status.
   *
   * @param index
   * @return Ptr<const StatusTimeseries> the pointer to the desired
   * status history.
   */
  virtual Ptr<const StatusTimeseries> get_status() const = 0;
};

/**
 * @brief This class implements the EncoderInterface.
 */
class Encoder : public EncoderInterface {
public:
  /**
   * @brief Construct a new Encoder object
   *
   * @param board is the EncoderBoard to be used.
   * @param encoder_id is the id of the motor on the on-board card
   */
  Encoder(Ptr<ControlBoardsInterface> board, JointNameIndexing encoder_id);

  /**
   * @brief Destroy the Encoder object
   *
   */
  virtual ~Encoder() {}

  /**
   * Getters
   */

  /**
   * @brief Get the measurements
   *
   * @param index is the kind of measurement we are instersted in.
   * see MeasurementIndex.
   * @return Ptr<const ScalarTimeseries> The history of the measurement
   */
  virtual Ptr<const ScalarTimeseries>
  get_measurement(const MeasurementIndex &index) const;

  /**
   * @brief Get the status.
   *
   * @param index
   * @return Ptr<const StatusTimeseries> the pointer to the desired
   * status history.
   */
  virtual Ptr<const StatusTimeseries> get_status() const;

  /** @brief Print the motor status and state. */
  virtual void print() const;

protected:
  /**
   * @brief The EncoderBoard to be used for the communication.
   */
  Ptr<ControlBoardsInterface> board_;

  /**
   * @brief The id of the motor on the EncoderBoard.
   */
  JointNameIndexing encoder_id_;
};

} // namespace monopod_drivers
