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



#include <monopod_sdk/blmc_drivers/blmc_joint_module.hpp>
#include <monopod_sdk/blmc_drivers/devices/device_interface.hpp>
#include <monopod_sdk/blmc_drivers/devices/motor.hpp>
#include "monopod_sdk/blmc_drivers/devices/analog_sensor.hpp"

#include "monopod_sdk/monopod_drivers/common_header.hpp"

namespace monopod_drivers
{
/**
 * @brief This class defines an interface to control a leg.
 * This leg is composed of 2 motors, one for the hip and one for the knee.
 */
class LegInterface
{
public:

    /**
     * @brief Enumerate the num_joints For readability
     *
     */
    static constexpr int num_joints_ = 2;

    /**
     * @brief MotorMeasurementIndexing this enum allow to access the different
     * kind of sensor measurements in an understandable way in the code.
     */
    enum MotorMeasurementIndexing
    {
        position = 1,
        velocity = 2,
        torque = 3,
        current = 4
    };


    /**
     * @brief Destroy the LegInterface object
     */
    virtual ~LegInterface()
    {
    }

    /**
     * Getters
     */

    /**
     * @brief Get the device output
     *
     * @param[in] measurement_index is th kind of data we are looking for.
     * @return LVector of measurements
     */
    virtual LVector get_measurements(const int& measurement_index) const = 0;


    /**
     * @brief Get the last sent target currentS.
     *
     * @return LVector is the list of the lasts time
     * stamped acquiered.
     */
    virtual LVector get_sent_current_targets() const = 0;


    /**
     * @brief Get the last sent target torque
     *
     * @param[in] joint_index designate the joint/motor from which we want the data
     * from.
     * @return LVector is the list of the lasts time
     * stamped acquiered.
     */
    virtual LVector get_sent_torque_targets() const = 0;

    /**
     * Setters
     */

    /**
     * @brief Set the torque targets for each joint.
     *
     * @param torque_target is the torque to achieve on the motor card.
     * @param joint_index is the motor to control.
     */
    virtual void set_target_torques(const LVector& torque_targets) = 0;


    /**
     * @brief Send the set torque
     */
    virtual void send_target_torques() = 0;

    /**
     * @brief Calibrate the leg. See blmc_joint_module.hpp for explanation of parameters
     * and logic.
     */
    virtual bool calibrate(const LVector& home_offset_rad) = 0;

    /**
     * @brief Converts torque to current
     *
     * @param torque
     * @return double
     */
    double joint_torque_to_motor_current(const LVector& torque) const;

    /**
     * @brief Converts torque to current
     *
     * @param torque
     * @return double
     */
    double joint_current_to_motor_torque(const LVector& current) const;
};

/**
 * @brief The leg class is the implementation of the LegInterface. This is
 * the decalartion and the definition of the class as it is very simple.
 */
class Leg : public LegInterface
{
public:

    /**
     * @brief Construct a new Leg object
     *
     * @param joints
     * @param motor_constant
     * @param gear_ratio
     */
    Leg(std::string can_bus_string, double motor_max_current=16.0)
    {

        motor_torque_constants_.setZero();
        joint_gear_ratios_.setZero();
        motor_max_current_.setZero();
        joint_zero_positions_.setZero();

        motor_torque_constants_.fill(0.025);
        joint_gear_ratios_.fill(9.0);
        motor_max_current_.fill(motor_max_current);

        can_bus_string_ = can_bus_string;
    }

    /**
     * @brief Destroy the Leg object
     */
    virtual ~Leg()
    {
    }

    /**
     * @brief Initialize the Leg class. establishes the Canbus and sets up the motorboards, etc.
     */
    void initialize()
    {

        can_bus_ = std::make_shared<blmc_drivers::CanBus>(can_bus_string_);
        can_motor_board_ = std::make_shared<blmc_drivers::CanBusMotorBoard>(can_bus_);

        motors_[hip_joint] = std::make_shared<blmc_drivers::SafeMotor>(
          can_motor_board_,
          0,
          motor_max_current_[hip_joint]);

        motors_[knee_joint] = std::make_shared<blmc_drivers::SafeMotor>(
          can_motor_board_,
          1,
          motor_max_current_[knee_joint]);

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
        can_motor_board_->wait_until_ready();

    }

    /// getters ================================================================

    /**
     * @brief Get specific measurements: pos, vel, torque, current. Returns
     * data for both joints.
     *
     * @param measurement_index
     * @return LVector
     */
      virtual LVector get_measurements(const int& measurement_index) const
      {
          switch(measurement_index)
          {
              case position:
              {
                  return joints_.get_measured_angles();
              }
              case velocity:
              {
                  return joints_.get_measured_velocities();
              }
              case torque:
              {
                  return joints_.get_measured_torques();
              }
              case current:
              {
                  LVector torques = joints_.get_measured_torques();
                  return joint_torque_to_motor_current(torques);
              }
              default:
              {
                  printf("Invalid measurement index passed");
                  LVector res(NAN, NAN);
                  return res;
              }
          }
      }

      /**
       * @brief Get all data returned as a matrix. Rows correspond to joints;
       * columns to position, velocity, torque.
       *
       * @return LMatrix (2, 3)
       */
      virtual LMatrix get_data()
      {
          LMatrix data;
          data.col(position - 1) = joints_.get_measured_angles();
          data.col(velocity - 1) = joints_.get_measured_velocities();
          data.col(torque - 1) = joints_.get_measured_torques();
          return data;
      }

      /**
       * @brief Get the sent current targets
       *
       * @return LVector
       */
      virtual LVector get_sent_current_targets() const
      {
          return joint_torque_to_motor_current(joints_.get_sent_torques());
      }

      /**
       * @brief Get the sent torque targets
       *
       * @return LVector
       */
      virtual LVector get_sent_torque_targets() const
      {
          return joints_.get_sent_torques();
      }

      /// setters ================================================================

      /**
       * @brief Set the target torques
       *
       * @param torque_targets
       */
      virtual void set_target_torques(const LVector& torque_targets)
      {
          joints_.set_torques(torque_targets);
      }

      /**
       * @brief Send target torques
       */
      virtual void send_target_torques(){
          joints_.send_torques();
      }

      /**
       * @brief Calibrate the legs
       *
       * @param home_offset_rad angle between zero position
       * @return true if calibration is successful
       */
      virtual bool calibrate(const LVector& home_offset_rad)
      {
          /**
           * TODO: double check search_distance_limit_rad
           */
          double search_distance_limit_rad = 2 * M_PI;
          LVector profile_step_size_rad = LVector::Constant(0.001);
          joints_.execute_homing(
              search_distance_limit_rad, home_offset_rad, profile_step_size_rad);
          LVector zero_pose = LVector::Zero();
          joints_.go_to(zero_pose);
          return true;
      }
      /// convert ================================================================

      /**
       * @brief Convert from motor current to joint torque.
       *
       * @param current is the motor current.
       * @return LVector of joint torques.
       */
      LVector motor_current_to_joint_torque(LVector currents) const
      {
          LVector torques;
          for(size_t i = 0; i < 2; i++)
          {
              torques[i] = currents[i] * joint_gear_ratios_[i] * motor_torque_constants_[i];
          }
          return torques;

      }

      /**
       * @brief Convert from joint torque to motor current
       *
       * @param torques
       * @return LVector of currents
       */
      LVector joint_torque_to_motor_current(LVector torques) const
      {
          LVector currents;
          for (unsigned i = 0; i < 2; ++i)
          {
              currents[i] = torques[i] / joint_gear_ratios_[i] / motor_torque_constants_[i];
          }
          return currents;
      }

private:

    /**
     * @brief string for can_bus.
     */
    std::string can_bus_string_;

    /**
     * @brief Canbus connection.
     */
    std::shared_ptr<blmc_drivers::CanBus> can_bus_;

    /**
     * @brief Canbus motorboard.
     */
    std::shared_ptr<blmc_drivers::CanBusMotorBoard> can_motor_board_;

    /**
     * @brief Hip and knee joint modules for the leg
     */
    blmc_drivers::BlmcJointModules<num_joints_> joints_;

    /**
    * @brief Hip and knee motor modules for the leg
    */
    std::array<std::shared_ptr<blmc_drivers::MotorInterface>, num_joints_> motors_;

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

    LVector joint_zero_positions_;

    LVector motor_max_current_;

};

}  // namespace blmc_drivers
