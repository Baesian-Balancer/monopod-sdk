
#include <time_series/time_series.hpp>
#include <planarizer.hpp>

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
            }

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
            }
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

            struct ReturnValueStatus
            {
                bool valid
                Ptr<const ScalarTimeSeries> value_series
            }

            struct PID
            {
                double p,
                double i,
                double d
            }
            /**
             * @brief Construct a new Monopod object
             * 
             * @param leg is the pointer to the leg
             * @param planarizer is the pointer to the planarizer
             */
            Monopod(std::shared_ptr<blmc_drivers::Leg> leg,
                    std::shared_ptr<monopod_drivers::planarizer> planarizer);

            
            /**
             * @brief Destroy the Monopod object
             */
            ~Monopod()

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

            /**
             * Setters
             */

            /**
             * @brief Set the PID values
             * 
             */
            void set_PID(const double &p_value, const double &i_value, const double &d_value) const
            
            /**
             * @brief Set the target torque and send command to monopod if value is different from
             * previous target_torque
             * 
             * @param joint_index 
             * @param torque_target 
             * @return ReturnValueStatus containing a valid boolean and the
             * value of the torque (NULL if not valid)
             */
            ReturnValueStatus set_target_torque(const int joint_index, const double &torque_target) const
        
        private:
            std::array<std::shared_ptr<DeviceInterface>, 2> monopod_;
    };

} // end monopod_drivers namespace