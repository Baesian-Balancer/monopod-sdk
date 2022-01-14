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

#include "monopod_sdk/monopod_drivers/devices/motor.hpp"
#include "monopod_sdk/monopod_drivers/utils/polynome.hpp"
#include "monopod_sdk/common_header.hpp"

namespace monopod_drivers
{

// TODO what is the best scope for those homing-related types?

/**
 * @brief Possible return values of the homing
 */
enum class HomingReturnCode
{
    //! Homing was not initialized and can therefore not be performed.
    NOT_INITIALIZED = 0,
    //! Homing is currently running.
    RUNNING,
    //! Homing is succeeded.
    SUCCEEDED,
    //! Homing failed.
    FAILED
};

/**
 * @brief Possible return values of the go_to
 */
enum class GoToReturnCode
{
    //! GoTo is currently running.
    RUNNING,
    //! Position has been reached succeeded.
    SUCCEEDED,
    //! Robot is stuck(hit an obstacle) before reaching its final position.
    FAILED
};

/**
 * @brief State variables required for the homing.
 */
struct HomingState
{
    //! Id of the joint.  Just used for debug prints.
    int joint_id = 0;
    //! Max. distance to move while searching the encoder index.
    double search_distance_limit_rad = 0.0;
    //! Offset from home position to zero position.
    double home_offset_rad = 0.0;
    //! Step size for the position profile.
    double profile_step_size_rad = 0.0;
    //! Timestamp from when the encoder index was seen the last time.
    long int last_encoder_index_time_index = 0;
    //! Number of profile steps already taken.
    uint32_t step_count = 0;
    //! Current target position of the position profile.
    double target_position_rad = 0.0;
    //! Current status of the homing procedure.
    HomingReturnCode status = HomingReturnCode::NOT_INITIALIZED;

    //! Position at which homing is started
    double start_position;
    /**
     * @brief Position at which homing is ended (before resetting position).
     *
     * This is only set when status is SUCCEEDED.  Together with start_position
     * it can be used to determine the distance the joint travelled during the
     * homing procedure (e.g. useful for home offset calibration).
     */
    double end_position;
};

/**
 * @brief The BlmcJointModule class is containing the joint information. It is
 * here to help converting the data from the motor side to the joint side. It
 * also allows the calibration of the joint position during initialization.
 */
class BlmcJointModule
{
public:
    /**
     * @brief Construct a new BlmcJointModule object
     *
     * @param motor is the C++ object allowing us to send commands and receive
     * sensor data.
     * @param motor_constant (\f$ k \f$) is the torque constant of the motor
     * \f$ \tau_{motor} = k * i_{motor} \f$
     * @param gear_ratio is the gear ratio between the motor and the joint.
     * @param zero_angle is the angle between the closest positive motor index
     * and the zero configuration.
     * @param reverse_polarity
     * @param max_current
     */
    BlmcJointModule(std::shared_ptr<monopod_drivers::MotorInterface> motor,
                    const double& motor_constant,
                    const double& gear_ratio,
                    const double& zero_angle,
                    const bool& reverse_polarity = false,
                    const double& max_current = 2.1);

    /**
     * @brief Set the joint torque to be sent.
     *
     * @param desired_torque (Nm)
     */
    void set_torque(const double& desired_torque);

    /**
     * @brief Set the zero_angle. The zero_angle is the angle between the
     * closest positive motor index and the zero configuration.
     *
     * @param zero_angle (rad)
     */
    void set_zero_angle(const double& zero_angle);

    /**
     * @brief Define if the motor should turn clock-wize or counter clock-wize.
     *
     * @param reverse_polarity true:reverse rotation axis, false:do nothing.
     */
    void set_joint_polarity(const bool& reverse_polarity);

    /**
     * @brief send the joint torque to the motor. The conversion between joint
     * torque and motor current is done automatically.
     */
    void send_torque();

    /**
     * @brief Get the maximum admissible joint torque that can be applied.
     *
     * @return double
     */
    double get_max_torque() const;

    /**
     * @brief Get the sent joint torque.
     *
     * @return double (Nm).
     */
    double get_sent_torque() const;

    /**
     * @brief Get the measured joint torque.
     *
     * @return double (Nm).
     */
    double get_measured_torque() const;

    /**
     * @brief Get the measured angle of the joint.
     *
     * @return double (rad).
     */
    double get_measured_angle() const;

    /**
     * @brief Get the measured velocity of the joint. This data is computed on
     * board of the control card.
     *
     * @return double (rad/s).
     */
    double get_measured_velocity() const;

    /**
     * @brief Get the measured index angle. There is one index per motor
     * rotation so there are gear_ratio indexes per joint rotation.
     *
     * @return double (rad).
     */
    double get_measured_index_angle() const;

    /**
     * @brief Get the zero_angle_. These are the angle between the starting pose
     * and the theoretical zero pose.
     *
     * @return double (rad).
     */
    double get_zero_angle() const;

    /**
     * @brief Set control gains for PD position controller.
     *
     * @param kp P gain ( (Nm) / rad ).
     * @param kd D gain ( (Nm) / (rad/s) ).
     */
    void set_position_control_gains(double kp, double kd);

    /**
     * @brief Execute one iteration of the position controller.
     *
     * @param target_position_rad  Target position (rad).
     *
     * @return Torque command (Nm).
     */
    double execute_position_controller(double target_position_rad) const;

    /**
     * @deprecated !!!!!!!
     * @brief This method calibrate the joint position knowing the angle between
     * the closest (in positive torque) motor index and the theoretical zero
     * pose. Warning, this method should be called in a real time thread!
     *
     * @param[in][out] angle_zero_to_index (rad) this is the angle between the
     * closest (in positive torque) motor index and the theoretical zero pose.
     * @param[out] index_angle (rad) is the angle where we met the index. This
     * angle is relative to the configuration when the robot booted.
     * @param[in] mechanical_calibration defines if the leg started in the zero
     * configuration or not
     * @return true if success.
     * @return false if problem arose.
     */
    bool calibrate(double& angle_zero_to_index,
                   double& index_angle,
                   bool mechanical_calibration = false);

    /**
     * @brief Set zero position relative to current position
     *
     * @param home_offset_rad  Offset from home position to zero position.
     *     Unit: radian.
     */
    void homing_at_current_position(double home_offset_rad);

    /**
     * @brief Initialize the homing procedure.
     *
     * This has to be called before update_homing().
     *
     * @param joint_id ID of the joint.  This is only used for debug prints.
     * @param search_distance_limit_rad  Maximum distance the motor moves while
     *     searching for the encoder index.  Unit: radian.
     * @param home_offset_rad  Offset from home position to zero position.
     *     Unit: radian.
     * @param profile_step_size_rad  Distance by which the target position of
     * the position profile is changed in each step.  Set to a negative value to
     *     search for the next encoder index in negative direction.  Unit:
     *     radian.
     */
    void init_homing(int joint_id,
                     double search_distance_limit_rad,
                     double home_offset_rad,
                     double profile_step_size_rad = 0.001);

    /**
     * @brief Perform one step of homing on encoder index.
     *
     * Searches for the next encoder index in positive direction and, when
     * found, sets it as home position.
     *
     * Only performs one step, so this method needs to be called in a loop. This
     * method only set the control, one *MUST* send the control for the motor
     * after calling this method.
     *
     * The motor is moved with a position profile until either the encoder index
     * is reached or the search distance limit is exceeded.  The position is
     * controlled with a simple PD controller.
     *
     * If the encoder index is found, its position is used as home position.
     * The zero position is offset from the home position by adding the "home
     * offset" to it (i.e. zero = home pos. + home offset).
     * If the search distance limit is reached before the encoder index occurs,
     * the homing fails.
     *
     * @return Status of the homing procedure.
     */
    HomingReturnCode update_homing();

    /**
     * @brief Get distance between start and end position of homing.
     *
     * Compute the distance that the joint moved between initialization of
     * homing and reaching the home position.
     *
     * This can be used to determine the home offset by first moving the joint
     * to the desired zero position, then executing the homing and finally
     * calling this function which will provide the desired home offset.
     *
     * @return Distance between start and end position of homing.
     */
    double get_distance_travelled_during_homing() const;

private:
    /**
     * @brief Convert from joint torque to motor current.
     *
     * @param[in] torque is the input joint
     * @return double the equivalent motor current.
     */
    double joint_torque_to_motor_current(double torque) const;

    /**
     * @brief Convert from motor current to joint torque.
     *
     * @param current is the motor current.
     * @return double is the equivalent joint torque.
     */
    double motor_current_to_joint_torque(double current) const;

    /**
     * @brief Get motor measurements and check if there are data or not.
     *
     * @param measurement_id is the id of the measurement you want to get.
     * check: monopod_drivers::MotorInterface::MeasurementIndex
     * @return double the measurement.
     */
    double get_motor_measurement(const MeasurementIndex& measurement_id) const;

    /**
     * @brief Get the last motor measurement index for a specific data. If there
     * was no data yet, return NaN
     *
     * @param measurement_id is the id of the measurement you want to get.
     * check: monopod_drivers::MotorInterface::MeasurementIndex
     * @return double the measurement.
     */
    long int get_motor_measurement_index(const MeasurementIndex& measurement_id) const;

    /**
     * @brief This is the pointer to the motor interface.
     */
    std::shared_ptr<monopod_drivers::MotorInterface> motor_;

    /**
     * @brief This is the torque constant of the motor:
     * \f$ \tau_{motor} = k * i_{motor} \f$
     */
    double motor_constant_;
    /**
     * @brief This correspond to the reduction (\f$ \beta \f$) between the motor
     * rotation and the joint. \f$ \theta_{joint} = \theta_{motor} / \beta \f$
     */
    double gear_ratio_;
    /**
     * @brief This is the distance between the closest positive index and the
     * zero configuration.
     */
    double zero_angle_;
    /**
     * @brief This change the motor rotation direction.
     */
    double polarity_;
    /**
     * @brief This is the maximum current we can apply during one experiment.
     * The program shut down if this value is achieved.
     */
    double max_current_;

    //! @brief P gain of the position PD controller.
    double position_control_gain_p_;
    //! @brief D gain of the position PD controller.
    double position_control_gain_d_;

    struct HomingState homing_state_;
};

}  // namespace monopod_drivers
