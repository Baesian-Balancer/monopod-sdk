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
#include <monopod_sdk/monopod_drivers/encoder_joint_module.hpp>

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
    Planarizer(std::shared_ptr<ControlBoardsInterface> board, const int &num_joints) :
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
        /*Always create at least one encoder on the main canbus.*/
        auto encoder_ppj = std::make_shared<monopod_drivers::Encoder>(
            board_,
            JointNameIndexing::planarizer_pitch_joint /* encoder id 0 */ );

        joints_[planarizer_pitch_joint] = std::make_shared<EncoderJointModule>(encoder_ppj,
                                                        1.0, //gear_ratios,
                                                        0.0, //zero_angles,
                                                        false);
        if (num_joints_ == 2 || num_joints_ == 3)
        {
              /*If num_joints_ == 2 we need to make a second encoder joint for meassurements. this is fixed hip mode*/
              auto encoder_pyj = std::make_shared<monopod_drivers::Encoder>(
                  board_,
                  JointNameIndexing::planarizer_yaw_joint /* encoder id 1*/ );

              joints_[planarizer_yaw_joint] = std::make_shared<EncoderJointModule>(encoder_pyj,
                                                              1.0, //gear_ratios,
                                                              0.0, //zero_angles,
                                                              false);
        }

        if (num_joints_ == 3)
        {      /*If num_joints_ == 3 we need to create second board. this is free hip mode*/

              auto encoder_bcj = std::make_shared<monopod_drivers::Encoder>(
                  board_,
                  JointNameIndexing::boom_connector_joint /* encoder id 0 */ );

              joints_[boom_connector_joint] = std::make_shared<EncoderJointModule>(encoder_bcj,
                                                              1.0, //gear_ratios,
                                                              0.0, //zero_angles,
                                                              false);
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

        for (const auto &pair : joints_)
        {
            data[pair.first][position] = pair.second->get_measured_angle();
            data[pair.first][velocity] = pair.second->get_measured_velocity();
        }

        return data;
    }

    /**
     * @brief Get the zero_angles. These are the joint angles between the
     * starting pose and the zero theoretical pose of the urdf.
     *
     * @return std::vector<double> (rad)
     */
    std::vector<double> get_zero_angles() const
    {

        std::vector<double> positions;
        positions.reserve(num_joints_);

        for (const auto &pair : joints_)
        {
            positions.push_back(pair.second->get_zero_angle());
        }
        return positions;

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

    /**
     * @brief Set the zero_angles. These are the joint angles between the
     * starting pose and the zero theoretical pose of the urdf.
     *
     * @param zero_angles (rad)
     */
    void set_zero_angles(const std::vector<double>& zero_angles)
    {
        if(zero_angles.size() != num_joints_)
            throw std::runtime_error("need same number of elements as number joints.");

        size_t i = 0;
        for (const auto &pair : joints_)
        {

            pair.second->set_zero_angle(zero_angles[i]);
            i++;
        }
    }

    /**
     * @brief Set the polarities of the joints
     * (see BlmcJointModule::set_joint_polarity)
     *
     * @param reverse_polarity
     */
    void set_joint_polarities(std::vector<bool> reverse_polarities)
    {
        if(reverse_polarities.size() != num_joints_)
            throw std::runtime_error("need same number of elements as number joints.");

        size_t i = 0;
        for (const auto &pair : joints_)
        {

            pair.second->set_joint_polarity(reverse_polarities[i]);
            i++;
        }
    }

private:

    /**
    * @brief Canbus board for encoders.
    */
    std::shared_ptr<monopod_drivers::ControlBoardsInterface> board_;

    /**
     * @brief number joints active.
     */
    long unsigned int num_joints_;

    /**
    * @brief Hip and knee motor modules for the Planarizer
    */
    std::unordered_map<JointNameIndexing, std::shared_ptr<monopod_drivers::EncoderJointModule>> joints_;

};

}  // namespace monopod_drivers
