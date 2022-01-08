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
// #include <time_series/time_series.hpp>

#include "monopod_sdk/blmc_drivers/devices/device_interface.hpp"
#include "monopod_sdk/blmc_drivers/devices/motor_board.hpp"
#include "monopod_sdk/monopod_drivers/common_header.hpp"

namespace monopod_drivers
{
/**
 * @brief This class declares an interface to the encoder. It allows the user to
 * access the encoder data.
 */
class EncoderInterface : public blmc_drivers::DeviceInterface
{
public:

    /**
     * @brief Here is a list of the different measurement available on the
     * blmc card.
     */
    enum MeasurementIndex
    {
        current,
        position,
        velocity,
        encoder_index,
        measurement_count
    };

    /**
     * @brief Destroy the MotorInterface object
     */
    virtual ~EncoderInterface()
    {
    }

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
    virtual Ptr<const ScalarTimeseries> get_measurement(
        const int& index = 0) const = 0;

};

/**
 * @brief This class implements the MotorInterface.
 */
class Encoder : public EncoderInterface
{
public:
    /**
     * @brief Construct a new Encoder object
     *
     * @param board is the MotorBoard to be used.
     * @param encoder_id is the id of the encoder on the on-board card
     */
    Encoder(Ptr<blmc_drivers::MotorBoardInterface> board, bool encoder_id);

    /**
     * @brief Destroy the Motor object
     *
     */
    virtual ~Encoder()
    {
    }

    /**
     * @brief Get the measurements
     *
     * @param index is the kind of measurement we are instersted in.
     * see MotorInterface::MeasurementIndex.
     * @return Ptr<const ScalarTimeseries> The history of the measurement
     */
    virtual Ptr<const ScalarTimeseries> get_measurement(
        const int& index = 0) const;


    /** @brief Print the  status and state. */
    virtual void print() const;

protected:
    /**
     * @brief The MotorBoard to be used for the communication.
     */
    Ptr<blmc_drivers::MotorBoardInterface> board_;

    /**
     * @brief The id of the encoder on the MotorBoard.
     */
    bool encoder_id_;
};


}  // namespace blmc_drivers
