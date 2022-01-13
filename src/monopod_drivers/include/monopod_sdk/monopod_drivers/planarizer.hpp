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



#include <monopod_sdk/blmc_drivers/blmc_joint_module.hpp>
#include <monopod_sdk/blmc_drivers/devices/device_interface.hpp>
#include <monopod_sdk/blmc_drivers/devices/motor.hpp>
#include "monopod_sdk/blmc_drivers/devices/analog_sensor.hpp"

#include "monopod_sdk/monopod_drivers/common_header.hpp"

namespace monopod_drivers
{

/**
 * @brief The Planarizer class is the implementation of the PlanarizerInterface. This is
 * the decalartion and the definition of the class as it is very simple.
 */

class Planarizer
{
public:

    typedef blmc_drivers::MotorInterface::MeasurementIndex mi;

    /**
     * @brief Construct the PlanarizerInterface object
     */
    Planarizer(const int &num_joints, const std::string &can_bus_id_0, const std::string &can_bus_id_1 = std::string()) :
      num_joints_(num_joints), joint_zero_positions_(num_joints, 0), polaritys_(num_joints, 1)
    {
        can_bus_id_0_ = can_bus_id_0;
        can_bus_id_1_ = can_bus_id_1;

        encoders_.reserve(num_joints_);

        // If no second can bus id was provided we can only have max of 2 joints.
        // 2 joints per can...
        if ((num_joints_ / 2) == (can_bus_id_1_.empty() ? 1 : 2))
          throw std::runtime_error("Number of canbus's is not compatible with the joints");

        if (!(num_joints_ == 1 || num_joints_ == 2 || num_joints_ == 3))
          throw std::runtime_error("Only support 1, 2, or 3 joints.");

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
        /*Always create the main canbus.*/
        can_bus_0_ = std::make_shared<blmc_drivers::CanBus>(can_bus_id_0_);
        can_encoder_board_0_ = std::make_shared<blmc_drivers::CanBusMotorBoard>(can_bus_0_);

        /*Always create at least one encoder on the main canbus.*/
        idx = monopod_drivers::JointModulesIndexMapping.at(planarizer_pitch_joint);
        encoders_[idx] = std::make_shared<blmc_drivers::Motor>(
            can_encoder_board_0_,
            0 /* encoder id 0 */ );

        if (num_joints_ == 2 || num_joints_ == 3)
        {
              /*If num_joints_ == 2 we need to make a second encoder joint for meassurements. this is fixed hip mode*/
              idx = monopod_drivers::JointModulesIndexMapping.at(planarizer_yaw_joint);
              encoders_[idx] = std::make_shared<blmc_drivers::Motor>(
                  can_encoder_board_0_,
                  1 /* encoder id 1*/ );
        }

        if (num_joints_ == 3)
        {      /*If num_joints_ == 3 we need to create second board. this is free hip mode*/
              can_bus_1_ = std::make_shared<blmc_drivers::CanBus>(can_bus_id_1_);
              can_encoder_board_1_ = std::make_shared<blmc_drivers::CanBusMotorBoard>(can_bus_1_);

              idx = monopod_drivers::JointModulesIndexMapping.at(boom_connector_joint);
              encoders_[idx] = std::make_shared<blmc_drivers::Motor>(
                  can_encoder_board_1_,
                  0 /* encoder id 0 */ );

              // wait until canbus 2 board is ready and connected
              can_encoder_board_1_->wait_until_ready();
        }

        // wait until canbus 1 board is ready and connected
        can_encoder_board_0_->wait_until_ready();

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
    std::unordered_map<int, std::vector<double>> get_measurements() const
    {
        std::unordered_map<int, std::vector<double>> data = {};

        std::vector<double> poss;
        std::vector<double> vels;

        poss.reserve(num_joints_);
        vels.reserve(num_joints_);

        for (size_t encoder_id = 0; encoder_id < encoders_.size(); encoder_id++){

            poss[encoder_id] = get_encoder_measurement(encoder_id, mi::position) - joint_zero_positions_[encoder_id];
            vels[encoder_id] = get_encoder_measurement(encoder_id, mi::velocity);
            // data[acceleration] = encoder_->get_measured_accelerations();
        }

        data[position] = poss;
        data[velocity] = vels;

        return data;
    }

    double get_encoder_measurement(const int &encoder_id, const mi& measurement_id) const
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
     * @brief number joints active.
     */
    int num_joints_;

    /**
     * @brief string for can_bus index 0.
     */
    std::string can_bus_id_0_;

    /**
     * @brief string for can_bus index 1.
     */
    std::string can_bus_id_1_;

    /**
     * @brief Canbus connection.
     */
    std::shared_ptr<blmc_drivers::CanBus> can_bus_0_;

    /**
     * @brief Canbus connection.
     */
    std::shared_ptr<blmc_drivers::CanBus> can_bus_1_;

    /**
    * @brief Canbus motorboard.
    */
    std::shared_ptr<blmc_drivers::CanBusMotorBoard> can_encoder_board_0_;

    /**
    * @brief Canbus motorboard.
    */
    std::shared_ptr<blmc_drivers::CanBusMotorBoard> can_encoder_board_1_;

    /**
    * @brief Hip and knee motor modules for the Planarizer
    */
    std::vector<std::shared_ptr<blmc_drivers::MotorInterface>> encoders_;

    /**
    * @brief Zero poisition for the joint.
    */
    std::vector<double> joint_zero_positions_;

    /**
    * @brief Sets the orientation of the meassurement for each joint
    */
    std::vector<double> polaritys_;

};

}  // namespace blmc_drivers
