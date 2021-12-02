#include <time_series/time_series.hpp>
#include <blmc_drivers/devices/analog_sensor.hpp>

namespace monopod_drivers
{
class Planarizer
{
    public:
        /**
         * @brief ScalarTimeseries is a simple shortcut for more intelligible code.
         */
        typedef time_series::TimeSeries<double> ScalarTimeseries;

        /**
         * @brief This is a shortcut for creating shared pointer in a simpler
         * writing expression.
         *
         * @tparam Type is the template paramer of the shared pointer.
         */
        template <typename Type>
        using Ptr = std::shared_ptr<Type>;

        struct ReturnValueStatus
        {
            bool valid
            Ptr<ScalarTimeSeries> value_series
        }

        enum PlanarizerIndexing
        {
            boom_connector,
            boom_yaw,
            boom_pitch
        }

        /**
         * @brief Construct a new Planarizer object
         * 
         * @param encoder_bc  boom connector encoder
         * @param encoder_by  boom yaw encoder
         * @param encoder_bp  boom pitch encoder
         */
        Planarizer(std::shared_ptr<blmc_drivers::EncoderInterface> encoder_bc,
                    std::shared_ptr<blmc_drivers::EncoderInterface> encoder_by,
                    std::shared_ptr<blmc_drivers::EncoderInterface> encoder_bp);
        
        /**
         * @brief Destroy the Planarizer object
         */
        ~Planarizer(){};

        /**
         * Getter 
         */
        ReturnValueStatus get_measurement(const int &joint_index,
                                                    const int &measurement_index) const;
    private:
        std::array<std::shared_ptr<EncoderInterface>, 3> encoders_;

};
} // end monopod_drivers namespace