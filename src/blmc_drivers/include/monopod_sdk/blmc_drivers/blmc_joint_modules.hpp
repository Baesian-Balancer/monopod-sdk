/**
 * @file blmc_joint_module.hpp
 * @author Manuel Wuthrich
 * @author Maximilien Naveau (maximilien.naveau@gmail.com)
 * @license License BSD-3-Clause
 * @copyright Copyright (c) 2019, New York University and Max Planck
 *            Gesellschaft.
 * @date 2019-07-11
 */
#pragma once

#include <math.h>
#include <array>
#include <iostream>
#include <stdexcept>

#include <Eigen/Eigen>

#include "monopod_sdk/blmc_drivers/devices/motor.hpp"
#include "monopod_sdk/blmc_drivers/utils/polynome.hpp"
#include <monopod_sdk/blmc_drivers/blmc_joint_module.hpp>

namespace blmc_drivers
{

/**
 * @brief BlmcJointModule_ptr shortcut for the shared pointer BlmcJointModule
 * type
 */
typedef std::shared_ptr<BlmcJointModule> BlmcJointModule_ptr;

/**
 * @brief This class defines an interface to a collection of BLMC joints. It
 * creates a BLMCJointModule for every blmc_driver::MotorInterface provided.
 *
 * @tparam COUNT
 */
template <int COUNT>
class BlmcJointModules
{
public:
    /**
     * @brief Defines a static Eigen vector type in order to define the
     * interface.
     */
    typedef Eigen::Matrix<double, COUNT, 1> Vector;

    /**
     * @brief Construct a new BlmcJointModules object.
     *
     * @param motors
     * @param motor_constants
     * @param gear_ratios
     * @param zero_angles
     */
    BlmcJointModules(
        const std::array<std::shared_ptr<blmc_drivers::MotorInterface>, COUNT>&
            motors,
        const Vector& motor_constants,
        const Vector& gear_ratios,
        const Vector& zero_angles,
        const Vector& max_currents)
    {
        set_motor_array(
            motors, motor_constants, gear_ratios, zero_angles, max_currents);
    }
    /**
     * @brief Construct a new BlmcJointModules object.
     */
    BlmcJointModules()
    {
    }
    /**
     * @brief Set the motor array, by creating the corresponding modules.
     *
     * @param motors
     * @param motor_constants
     * @param gear_ratios
     * @param zero_angles
     */
    void set_motor_array(
        const std::array<std::shared_ptr<blmc_drivers::MotorInterface>, COUNT>&
            motors,
        const Vector& motor_constants,
        const Vector& gear_ratios,
        const Vector& zero_angles,
        const Vector& max_currents)
    {
        for (size_t i = 0; i < COUNT; i++)
        {
            modules_[i] = std::make_shared<BlmcJointModule>(motors[i],
                                                            motor_constants[i],
                                                            gear_ratios[i],
                                                            zero_angles[i],
                                                            false,
                                                            max_currents[i]);
        }
    }
    /**
     * @brief Send the registered torques to all modules.
     */
    void send_torques()
    {
        for (size_t i = 0; i < COUNT; i++)
        {
            modules_[i]->send_torque();
        }
    }

    /**
     * @brief Set the polarities of the joints
     * (see BlmcJointModule::set_joint_polarity)
     *
     * @param reverse_polarity
     */
    void set_joint_polarities(std::array<bool, COUNT> reverse_polarities)
    {
        for (size_t i = 0; i < COUNT; i++)
        {
            modules_[i]->set_joint_polarity(reverse_polarities[i]);
        }
    }
    /**
     * @brief Register the joint torques to be sent for all modules.
     *
     * @param desired_torques (Nm)
     */
    void set_torques(const Vector& desired_torques)
    {
        for (size_t i = 0; i < COUNT; i++)
        {
            modules_[i]->set_torque(desired_torques(i));
        }
    }

    /**
     * @brief Get the maximum admissible joint torque that can be applied.
     *
     * @return Vector (N/m)
     */
    Vector get_max_torques()
    {
        Vector max_torques;
        for (size_t i = 0; i < COUNT; ++i)
        {
            max_torques[i] = modules_[i]->get_max_torque();
        }
        return max_torques;
    }

    /**
     * @brief Get the previously sent torques.
     *
     * @return Vector (Nm)
     */
    Vector get_sent_torques() const
    {
        Vector torques;

        for (size_t i = 0; i < COUNT; i++)
        {
            torques(i) = modules_[i]->get_sent_torque();
        }
        return torques;
    }

    /**
     * @brief Get the measured joint torques.
     *
     * @return Vector (Nm)
     */
    Vector get_measured_torques() const
    {
        Vector torques;

        for (size_t i = 0; i < COUNT; i++)
        {
            torques(i) = modules_[i]->get_measured_torque();
        }
        return torques;
    }

    /**
     * @brief Get the measured joint angles.
     *
     * @return Vector (rad)
     */
    Vector get_measured_angles() const
    {
        Vector positions;

        for (size_t i = 0; i < COUNT; i++)
        {
            positions(i) = modules_[i]->get_measured_angle();
        }
        return positions;
    }

    /**
     * @brief Get the measured joint velocities.
     *
     * @return Vector (rad/s)
     */
    Vector get_measured_velocities() const
    {
        Vector velocities;

        for (size_t i = 0; i < COUNT; i++)
        {
            velocities(i) = modules_[i]->get_measured_velocity();
        }
        return velocities;
    }

    /**
     * @brief Set the zero_angles. These are the joint angles between the
     * starting pose and the zero theoretical pose of the urdf.
     *
     * @param zero_angles (rad)
     */
    void set_zero_angles(const Vector& zero_angles)
    {
        for (size_t i = 0; i < COUNT; i++)
        {
            modules_[i]->set_zero_angle(zero_angles(i));
        }
    }
    /**
     * @brief Get the zero_angles. These are the joint angles between the
     * starting pose and the zero theoretical pose of the urdf.
     *
     * @return Vector (rad)
     */
    Vector get_zero_angles() const
    {
        Vector positions;

        for (size_t i = 0; i < COUNT; i++)
        {
            positions(i) = modules_[i]->get_zero_angle();
        }
        return positions;
    }
    /**
     * @brief Get the index_angles. There is one index per motor rotation so
     * there are gear_ratio indexes per joint rotation.
     *
     * @return Vector (rad)
     */
    Vector get_measured_index_angles() const
    {
        Vector index_angles;

        for (size_t i = 0; i < COUNT; i++)
        {
            index_angles(i) = modules_[i]->get_measured_index_angle();
        }
        return index_angles;
    }

    /**
     * @brief Set position control gains for the specified joint.
     *
     * @param joint_id  ID of the joint (in range `[0, COUNT)`).
     * @param kp P gain.
     * @param kd D gain.
     */
    void set_position_control_gains(size_t joint_id, double kp, double kd)
    {
        modules_[joint_id]->set_position_control_gains(kp, kd);
    }

    /**
     * @brief Set position control gains for all joints.
     *
     * @param kp P gains.
     * @param kd D gains.
     */
    void set_position_control_gains(Vector kp, Vector kd)
    {
        for (size_t i = 0; i < COUNT; i++)
        {
            set_position_control_gains(i, kp[i], kd[i]);
        }
    }

    /**
     * @brief Perform homing for all joints at endstops.
     *
     * See BlmcJointModule::homing_at_current_position for description of the
     * arguments.
     *
     * @return Final status of the homing procedure (since homing happens at
     * current,position, procedure always returns success).
     */
    HomingReturnCode execute_homing_at_current_position(Vector home_offset_rad)
    {
        // Initialise homing for all joints
        for (size_t i = 0; i < COUNT; i++)
        {
            modules_[i]->homing_at_current_position(home_offset_rad[i]);
        }

        return HomingReturnCode::SUCCEEDED;
    }

    /**
     * @brief Perform homing for all joints.
     *
     * If one of the joints fails, the complete homing fails.  Otherwise it
     * loops until all joints finished.
     * If a joint is finished while others are still running, it is held at the
     * home position.
     *
     * See BlmcJointModule::update_homing for details on the homing procedure.
     *
     * See BlmcJointModule::init_homing for description of the arguments.
     *
     * @return Final status of the homing procedure (either SUCCESS if all
     *     joints succeeded or the return code of the first joint that failed).
     */
    HomingReturnCode execute_homing(
        double search_distance_limit_rad,
        Vector home_offset_rad,
        Vector profile_step_size_rad = Vector::Constant(0.001))
    {
        // Initialise homing for all joints
        for (size_t i = 0; i < COUNT; i++)
        {
            modules_[i]->init_homing((int)i,
                                     search_distance_limit_rad,
                                     home_offset_rad[i],
                                     profile_step_size_rad[i]);
        }

        // run homing for all joints until all of them are done
        real_time_tools::Spinner spinner;
        spinner.set_period(0.001);  // TODO magic number
        HomingReturnCode homing_status;
        do
        {
            bool all_succeeded = true;
            homing_status = HomingReturnCode::RUNNING;

            for (size_t i = 0; i < COUNT; i++)
            {
                HomingReturnCode joint_result = modules_[i]->update_homing();

                all_succeeded &= (joint_result == HomingReturnCode::SUCCEEDED);

                if (joint_result == HomingReturnCode::NOT_INITIALIZED ||
                    joint_result == HomingReturnCode::FAILED)
                {
                    homing_status = joint_result;
                    // abort homing
                    break;
                }
            }
            if (homing_status == HomingReturnCode::RUNNING)
            {
                for (unsigned i = 0; i < COUNT; ++i)
                {
                    modules_[i]->send_torque();
                }
            }

            if (all_succeeded)
            {
                homing_status = HomingReturnCode::SUCCEEDED;
            }

            spinner.spin();
        } while (homing_status == HomingReturnCode::RUNNING);

        return homing_status;
    }

    //! @see BlmcJointModule::get_distance_travelled_during_homing
    Vector get_distance_travelled_during_homing() const
    {
        Vector dist;
        for (unsigned i = 0; i < COUNT; ++i)
        {
            dist[i] = modules_[i]->get_distance_travelled_during_homing();
        }
        return dist;
    }

    /**
     * @brief Allow the robot to go to a desired pose. Once the control done
     * 0 torques is sent.
     *
     * @param angle_to_reach_rad (rad)
     * @param average_speed_rad_per_sec (rad/sec)
     * @return GoToReturnCode
     */
    GoToReturnCode go_to(Vector angle_to_reach_rad,
                         double average_speed_rad_per_sec = 1.0)
    {
        // Compute a min jerk trajectory
        Vector initial_joint_positions = get_measured_angles();
        double final_time = (angle_to_reach_rad - initial_joint_positions)
                                .array()
                                .abs()
                                .maxCoeff() /
                            average_speed_rad_per_sec;

        std::array<TimePolynome<5>, COUNT> min_jerk_trajs;
        for (unsigned i = 0; i < COUNT; i++)
        {
            min_jerk_trajs[i].set_parameters(final_time,
                                             initial_joint_positions[i],
                                             0.0 /*initial speed*/,
                                             angle_to_reach_rad[i]);
        }

        // run got_to for all joints
        real_time_tools::Spinner spinner;
        double sampling_period = 0.001;  // TODO magic number
        spinner.set_period(sampling_period);
        GoToReturnCode go_to_status;
        double current_time = 0.0;
        do
        {
            // TODO: add a security if error gets too big
            for (unsigned i = 0; i < COUNT; i++)
            {
                double desired_pose = min_jerk_trajs[i].compute(current_time);
                double desired_torque =
                    modules_[i]->execute_position_controller(desired_pose);
                modules_[i]->set_torque(desired_torque);
            }
            for (unsigned i = 0; i < COUNT; ++i)
            {
                modules_[i]->send_torque();
            }
            go_to_status = GoToReturnCode::RUNNING;

            current_time += sampling_period;
            spinner.spin();

        } while (current_time < (final_time + sampling_period));

        // Stop all motors (0 torques) after the destination achieved
        for (unsigned i = 0; i < COUNT; i++)
        {
            modules_[i]->set_torque(0.0);
        }
        for (unsigned i = 0; i < COUNT; ++i)
        {
            modules_[i]->send_torque();
        }

        Vector final_pos = get_measured_angles();
        if ((angle_to_reach_rad - final_pos).isMuchSmallerThan(1.0, 1e-3))
        {
            go_to_status = GoToReturnCode::SUCCEEDED;
        }
        else
        {
            go_to_status = GoToReturnCode::FAILED;
        }
        return go_to_status;
    }

private:
    /**
     * @brief These are the BLMCJointModule objects corresponding to a robot.
     */
    std::array<std::shared_ptr<BlmcJointModule>, COUNT> modules_;
};

}  // namespace blmc_drivers
