/**
 * @file Planarizer.hpp
 * @license License BSD-3-Clause
 * @copyright Copyright (c) 2019, New York University and Max Planck
 * Gesellschaft.
 * @date 2019-07-11
 */

#pragma once

#include <map>
#include <memory>
#include <string>
#include <math.h>

#include <time_series/time_series.hpp>

#include <monopod_sdk/monopod_drivers/devices/device_interface.hpp>
#include <monopod_sdk/monopod_drivers/devices/motor.hpp>

#include "monopod_sdk/common_header.hpp"

namespace monopod_drivers
{

/**
 * @brief The Planarizer class is the implementation of the PlanarizerInterface. This is
 * the decalartion and the definition of the class as it is very simple.
 */

class Planarizer
{
public:

    /**
     * @brief Construct the PlanarizerInterface object
     */
    Planarizer(std::shared_ptr<MotorBoardInterface> board, const int &num_joints) :
      board_(board), num_joints_(num_joints)
    {

        if (!(num_joints_ == 1 || num_joints_ == 2 || num_joints_ == 3))
            throw std::runtime_error("Only support 1 (fixed), 2 (fixed_hip), or 3 joints (free_hip).");
    }

    /**
     * @brief Destroy the PlanarizerInterface object
     */
    ~Planarizer()
    {
    }

    /**
    * @brief Initialize canbus connecion, esablish connection to the motors, and set
    * motor constants.
    */
    bool initialize()
    {
        int idx;
        /*Always create at least one encoder on the main canbus.*/
        encoders_[planarizer_pitch_joint] = std::make_shared<monopod_drivers::Encoder>(
            board_,
            JointNameIndexing::planarizer_pitch_joint /* encoder id 0 */ );

        if (num_joints_ == 2 || num_joints_ == 3)
        {
              /*If num_joints_ == 2 we need to make a second encoder joint for meassurements. this is fixed hip mode*/
              encoders_[planarizer_yaw_joint] = std::make_shared<monopod_drivers::Encoder>(
                  board_,
                  JointNameIndexing::planarizer_yaw_joint /* encoder id 1*/ );
        }

        if (num_joints_ == 3)
        {      /*If num_joints_ == 3 we need to create second board. this is free hip mode*/

              encoders_[boom_connector_joint] = std::make_shared<monopod_drivers::Encoder>(
                  board_,
                  JointNameIndexing::boom_connector_joint /* encoder id 0 */ );

              // wait until canbus 2 board is ready and connected
              board_->wait_until_ready();
        }

        // wait until canbus 1 board is ready and connected
        board_->wait_until_ready();

        return true;

    }

    // =========================================================================
    //  GETTERS
    // =========================================================================

    /**
     * @brief Get all meassurements of the Planarizer. This includes Position, Velocity,
     * and In the future Acceleration.
     *
     * @return unordered map of std::vector<double> measurements. Indexed with the meassurement type enum.
     */
    ObservationMap get_measurements() const
    {

        ObservationMap data = {};

        for (const auto &pair : encoders_)
        {
            data[pair.first][position] = get_encoder_measurement(encoder_id, position) - joint_zero_positions_[encoder_id];
            data[pair.first][velocity] = get_encoder_measurement(encoder_id, velocity);
        }

        return data;
    }

    double get_encoder_measurement(const int &encoder_id, const MeasurementIndex& measurement_id) const
    {
        auto measurement_history = encoders_[encoder_id]->get_measurement(measurement_id);

        if (measurement_history->length() == 0)
        {
            return std::numeric_limits<double>::quiet_NaN();
        }

        return polaritys_[encoder_id] * measurement_history->newest_element();
    }

    // =========================================================================
    //  SETTERS
    // =========================================================================

    /**
     * @brief Calibrate the Planarizer. See blmc_joint_module.hpp for explanation of parameters
     * and logic.
     */
    bool calibrate(const std::vector<double>& home_offset_rad)
    {
        return true;
    }


private:

    /**
    * @brief Canbus board for encoders.
    */
    std::shared_ptr<monopod_drivers::MotorBoardInterface> board_;

    /**
     * @brief number joints active.
     */
    int num_joints_;

    /**
    * @brief Hip and knee motor modules for the Planarizer
    */
    std::unordered_map<JointNameIndexing, std::shared_ptr<monopod_drivers::EncoderInterface>> encoders_;

    /**
    * @brief Zero poisition for the joint.
    */
    std::vector<double> joint_zero_positions_;

    /**
    * @brief Sets the orientation of the meassurement for each joint
    */
    std::vector<double> polaritys_;

};

}  // namespace monopod_drivers
