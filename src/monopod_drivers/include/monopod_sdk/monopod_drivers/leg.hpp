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

#include <time_series/time_series.hpp>

#include <monopod_sdk/blmc_drivers/blmc_joint_module.hpp>
#include <monopod_sdk/blmc_drivers/devices/device_interface.hpp>
#include <monopod_sdk/blmc_drivers/devices/motor.hpp>

namespace monopod_drivers
{
/**
 * @brief This class defines an interface to control a leg.
 * This legg is composed of 2 motor, one for the hip and one for the knee.
 */
class LegInterface
{
public:
    
    /**
     * @brief Defines a static Eigen vector type in order to define the
     * interface. Two is for number of joints
     */
    typedef Eigen::Matrix<double, 2, 1> Vector;

    /**
     * @brief Defines a static Eigen vector type in order to define the
     * interface. Two is for number of joints, 3 is for pos, vel, torque
     */
    typedef Eigen::Matrix<double, 2, 3> DVector;

    /**
     * @brief Enumerate the num_joints For readability
     * 
     */
    static constexpr int num_joints_ = 2; 

    /**
     * @brief This is a shortcut for creating shared pointer in a simpler
     * writting expression.
     *
     * @tparam Type is the template paramer of the shared pointer.
     */
    template <typename Type>
    using Ptr = std::shared_ptr<Type>;

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
     * @brief This enum list the motors in the leg
     */
    enum MotorIndexing
    {
        hip,
        knee,
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
     * @return Vector of measurements
     */
    virtual Vector get_measurements(const int& measurement_index) const = 0;

 
    /**
     * @brief Get the last sent target currentS.
     *
     * @return Vector is the list of the lasts time
     * stamped acquiered.
     */
    virtual Vector get_sent_current_targets() const = 0;


    /**
     * @brief Get the last sent target torque
     *
     * @param[in] joint_index designate the joint/motor from which we want the data
     * from.
     * @return Vector is the list of the lasts time
     * stamped acquiered.
     */
    virtual Vector get_sent_torque_targets() const = 0;

    /**
     * Setters
     */

    /**
     * @brief Set the torque targets for each joint. Sends if input is different
     * 
     * @param torque_target is the current to achieve on the motor card.
     * @param joint_index is the motor to control.
     */
    virtual void set_target_torques(const Vector& torque_targets) = 0;

    /**
     * @brief Converts torque to current
     * 
     * @param torque 
     * @return double 
     */
    double joint_torque_to_motor_current(const Vector& torque) const;

    /**
     * @brief Converts torque to current
     * 
     * @param torque 
     * @return double 
     */
    double joint_current_to_motor_torque(const Vector& current) const;
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
    Leg(blmc_drivers::BlmcJointModules<2> joints,
        const Vector& motor_constants,
        const Vector& gear_ratios)
    {
        motor_constants_ = motor_constants;
        gear_ratios_ = gear_ratios;
        joints_ = joints;

    }

    /**
     * @brief Destroy the Leg object
     */
    virtual ~Leg()
    {
    }

    /**
     * Getters
     */

    
    /**
     * @brief Get specific measurements for both joints: pos, vel, torque, current.
     * 
     * @param measurement_index
     * @return Vector
     */
    virtual Vector get_measurements(const int& measurement_index) const
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
                Vector torques = joints_.get_measured_torques();
                return joint_torque_to_motor_current(torques);
            }
            default:
            {
                printf("Invalid measurement index passed");
                Vector res(NAN, NAN);
                return res;
            }
        }
    }

    /**
     * @brief Get all data returned as a matrix. Rows correspond
     * to joints; columns to position, velocity, torque.
     * @return DVector (2, 3)
     */
    virtual DVector get_data()
    {
        DVector data;
        data.col(position - 1) = joints_.get_measured_angles();
        data.col(velocity - 1 ) = joints_.get_measured_velocities();
        data.col(torque - 1) = joints_.get_measured_torques();
        return data;
    }

    // input logs --------------------------------------------------------------
    
    virtual Vector get_sent_current_targets() const
    {
        return joint_torque_to_motor_current(joints_.get_sent_torques());
    }  

    virtual Vector get_sent_torque_targets() const
    {
        return joints_.get_sent_torques();
    }

    /// setter ================================================================
    virtual void set_target_torques(const Vector& torque_targets)
    {
        joints_.set_torques(torque_targets);
    }

    Vector joint_torque_to_motor_current(Vector torques) const
    {
        Vector currents; 
        for (unsigned i = 0; i < 2; ++i)
        {
            currents[i] = torques[i] / gear_ratios_[i] / motor_constants_[i];
        }
        return currents;
    }

    void send_target_torques(){
        joints_.send_torques();
    }

    /**
     * @brief Convert from motor current to joint torque.
     *
     * @param current is the motor current.
     * @return double is the equivalent joint torque.
     */
    Vector motor_current_to_joint_torque(Vector currents) const
    {
        Vector torques;
        for(size_t i = 0; i < 2; i++)
        {
            torques[i] = currents[i] * gear_ratios_[i] * motor_constants_[i];
        }
        return torques;

    }

    

private:

    /**
     * @brief Hip and knee joint modules for the leg
     */

    blmc_drivers::BlmcJointModules<2> joints_;

    /**
     * @brief This is the torque constant of the motor:
     * \f$ \tau_{motor} = k * i_{motor} \f$
     */
    Vector motor_constants_;
    /**
     * @brief This correspond to the reduction (\f$ \beta \f$) between the motor
     * rotation and the joint. \f$ \theta_{joint} = \theta_{motor} / \beta \f$
     */
    Vector gear_ratios_;

    

};

}  // namespace blmc_drivers
