
#pragma once

#include <time_series/time_series.hpp>
#include <monopod_sdk/monopod_drivers/planarizer.hpp>
#include <monopod_sdk/monopod_drivers/leg.hpp>

/**
 * @brief The Monopod class defines the API for interacting with the Baesian
 * monopod. It takes a leg
 *
 */
namespace monopod_drivers
{
    class Monopod
    {
    public:
        /**
             * @brief ScalarTimeseries is a simple shortcut for more intelligible code.
             */
        typedef time_series::TimeSeries<double> ScalarTimeseries;

        /**
             * @brief This is a shortcut for creating shared pointer in a simpler
             * writting expression.
             *
             * @tparam Type is the template paramer of the shared pointer.
             */
        template <typename Type>
        using Ptr = std::shared_ptr<Type>;

        enum MonopodBodyIndexing
        {
            leg,
            planarizer
        };

        /**
             * @brief Enumerates the joint names for indexing
             */
        enum JointNameIndexing
        {
            hip,
            knee,
            boom_connector,
            boom_yaw,
            boom_pitch
        };
        /**
             * @brief MotorMeasurementIndexing this enum allow to access the different
             * kind of sensor measurements in an understandable way in the code.
             *
             * Note: This is same as in leg.hpp for consistency
             */
        enum MotorMeasurementIndexing
        {
            current,
            position,
            velocity,
            encoder_index,
            motor_measurement_count
        };

        // struct ReturnValueStatus
        // {
        //     bool valid;
        //     Ptr<const ScalarTimeseries> value_series;
        // };
        struct ReturnValueStatus
        {
            bool valid;
            double value_series;
        };

        struct PID
        {
            double p;
            double i;
            double d;
        };
        /**
             * @brief Construct a new Monopod object
             *
             * @param leg is the pointer to the leg
             * @param planarizer is the pointer to the planarizer
             */
        Monopod(std::shared_ptr<Leg> leg_,
                std::shared_ptr<Planarizer> planarizer_);

        /**
             * @brief Destroy the Monopod object
             */
        ~Monopod();

        /**
             * Getters
             */

        /**
             * @brief Get the position of joint
             *
             * @param joint_index name of the joint we want to access
             * @return ReturnValueStatus containing a valid boolean and the
             * value of the position (NULL if not valid)
             */
        ReturnValueStatus get_position(const int joint_index);

        /**
             * @brief Get the velocity of the joint
             *
             * @param joint_index name of the joint we want to access
             * @return ReturnValueStatus containing a valid boolean and the
             * value of the velocity (NULL if not valid)
             */
        ReturnValueStatus get_velocity(const int joint_index);

        /**
             * @brief Get the acceleration of the joint
             *
             * @param joint_index name of the joint we want to access
             * @return ReturnValueStatus containing a valid boolean and the
             * value of the acceleration (NULL if not valid)
             */
        ReturnValueStatus get_acceleration(const int joint_index);

        /**
             * @brief Get the PID values
             *
             * @param joint_index
             * @return PID struct containing PID values
             */
        PID get_PID();

        std::vector<std::string> get_joint_indexing() const;

        /**
             * Setters
             */

        /**
             * @brief Set the PID values
             *
             */
        void set_PID(const double &p_value, const double &i_value, const double &d_value);

        /**
             * @brief Set the target torque and send command to monopod if value is different from
             * previous target_torque
             *
             * @param joint_index
             * @param torque_target
             * @return ReturnValueStatus containing a valid boolean and the
             * value of the torque (NULL if not valid)
             */
        ReturnValueStatus set_target_torque(const int joint_index, const double &torque_target);

        const std::vector<std::string> joint_str_indexer{"hip", "knee", "boom_connector", "boom_yaw", "boom_pitch"};


    private:
        // std::array<std::shared_ptr<MonopodSubInterface>, 2> monopod_;
        std::shared_ptr<Leg> leg_;
        std::shared_ptr<Planarizer> planarizer_;

        ReturnValueStatus return_value_status_;

        PID pid_;
    };

} // end monopod_drivers namespace
