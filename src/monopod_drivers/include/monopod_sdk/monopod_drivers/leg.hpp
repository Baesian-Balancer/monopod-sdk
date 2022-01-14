/**
 * @file leg.hpp
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



#include <monopod_sdk/monopod_drivers/motor_joint_modules.hpp>
#include <monopod_sdk/monopod_drivers/devices/device_interface.hpp>
#include <monopod_sdk/monopod_drivers/devices/motor.hpp>

#include "monopod_sdk/common_header.hpp"

namespace monopod_drivers
{


/**
 * @brief The leg class is the implementation of the LegInterface. This is
 * the decalartion and the definition of the class as it is very simple.
 */
class Leg
{
public:
    /**
     * @brief Enumerate the num_joints For readability
     *
     */
    static constexpr int num_joints_ = 2;

    /**
     * @brief Construct the LegInterface object
     */
    Leg(std::shared_ptr<MotorBoardInterface> board, double motor_max_current = 5.0)
      : board_(board)
    {

        motor_torque_constants_.setZero();
        joint_gear_ratios_.setZero();
        motor_max_current_.setZero();
        joint_zero_positions_.setZero();

        motor_torque_constants_.fill(0.025);
        joint_gear_ratios_.fill(9.0);
        motor_max_current_.fill(motor_max_current);

    }

    /**
     * @brief Destroy the LegInterface object
     */
    ~Leg()
    {
    }

    /**
     * @brief Initialize canbus connecion, esablish connection to the motors, and set
     * motor constants.
     */
    bool initialize()
    {

        motors_[hip_joint] = std::make_shared<monopod_drivers::SafeMotor>(
          board_,
          hip_joint,
          motor_max_current_[hip_joint] * 0.99); // 0.99 means that safe motor will have a chance to limit current before

        motors_[knee_joint] = std::make_shared<monopod_drivers::SafeMotor>(
          board_,
          knee_joint,
          motor_max_current_[knee_joint] * 0.99);

        // Create the joint module objects
        joints_.set_motor_array(motors_,
                                motor_torque_constants_,
                                joint_gear_ratios_,
                                joint_zero_positions_,
                                motor_max_current_);

        // The the control gains in order to perform the calibration
        LVector kp, kd;
        kp.fill(3.0);
        kd.fill(0.05);
        joints_.set_position_control_gains(kp, kd);

        // wait until all board are ready and connected
        board_->wait_until_ready();

        return true;

    }

    // =========================================================================
    //  GETTERS
    // =========================================================================

    /**
     * @brief Get all meassurements of the leg. This includes Position, Velocity,
     * Torque, and In the future Acceleration.
     *
     * @return unordered map of LVector measurements. Indexed with the meassurement type enum.
     */
    std::unordered_map<int, LVector> get_measurements() const
    {
        std::unordered_map<int, LVector> data = {};
        data[position] = joints_.get_measured_angles();
        data[velocity] = joints_.get_measured_velocities();
        // data[acceleration] = joints_.get_measured_accelerations();
        data[torque] = joints_.get_measured_torques();
        return data;
    }

    // =========================================================================
    //  SETTERS
    // =========================================================================

    /**
     * @brief Set the torque targets for each joint.
     *
     * @param torque_target is the torque to achieve on the motor card.
     * @param joint_index is the motor to control.
     */
    void set_target_torques(const LVector& torque_targets)
    {
        joints_.set_torques(torque_targets);
    }

    /**
     * @brief Send the set torque
     */
    void send_target_torques()
    {
        joints_.send_torques();
    }

    /**
     * @brief Calibrate the leg. See blmc_joint_module.hpp for explanation of parameters
     * and logic.
     */
    bool calibrate(const LVector& home_offset_rad)
    {
        double search_distance_limit_rad = 2 * M_PI;
        LVector profile_step_size_rad = LVector::Constant(0.001);
        joints_.execute_homing(
            search_distance_limit_rad, home_offset_rad, profile_step_size_rad);
        LVector zero_pose = LVector::Zero();
        joints_.go_to(zero_pose);
        return true;
    }

private:


    /**
     * @brief Canbus motorboard.
     */
    std::shared_ptr<monopod_drivers::MotorBoardInterface> board_;

    /**
     * @brief Hip and knee joint modules for the leg
     */
    monopod_drivers::MotorJointModules<num_joints_> joints_;

    /**
    * @brief Hip and knee motor modules for the leg
    */
    std::array<std::shared_ptr<monopod_drivers::MotorInterface>, num_joints_> motors_;

    /**
     * @brief This is the torque constant of the motor:
     * \f$ \tau_{motor} = k * i_{motor} \f$
     */
    LVector motor_torque_constants_;

    /**
     * @brief This correspond to the reduction (\f$ \beta \f$) between the motor
     * rotation and the joint. \f$ \theta_{joint} = \theta_{motor} / \beta \f$
     */
    LVector joint_gear_ratios_;

    /**
    * @brief Zero poisition for the joint.
    */
    LVector joint_zero_positions_;

    /**
    * @brief Max allowable current for the joint.
    */
    LVector motor_max_current_;
};

}  // namespace monopod_drivers
