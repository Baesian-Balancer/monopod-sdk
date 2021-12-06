
#include "monopod_sdk/monopod_drivers/planarizer.hpp"
#include "monopod_sdk/monopod_drivers/monopod.hpp"


namespace monopod_drivers
{
    /**
     * @brief ScalarTimeseries is a simple shortcut for more intelligible code.
     */
    typedef time_series::TimeSeries<double> ScalarTimeseries;

    /**
     * @brief This is a shortcut for creating shared pointer in a simpler
     * writing expression.
     *
     * @tparam Type is the template parameter of the shared pointer.
     */
    template <typename Type>
    using Ptr = std::shared_ptr<Type>;
    
    Planarizer::Planarizer(std::shared_ptr<monopod_drivers::EncoderInterface> encoder_bc,
            std::shared_ptr<monopod_drivers::EncoderInterface> encoder_by,
            std::shared_ptr<monopod_drivers::EncoderInterface> encoder_bp)
    {
        encoders_[Monopod::JointNameIndexing::boom_connector] = encoder_bc;
        encoders_[Monopod::JointNameIndexing::boom_yaw] = encoder_by;
        encoders_[Monopod::JointNameIndexing::boom_pitch] = encoder_bp;
    }

    Ptr<const ScalarTimeseries> Planarizer::get_measurement(const int &joint_index,
                                        const int &measurement_index) const
    {
        return encoders_[joint_index]->get_measurement(measurement_index);
    }

} // end monopod_drivers namespace
