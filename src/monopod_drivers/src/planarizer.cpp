
#include "monopod_sdk/monopod_drivers/planarizer.hpp"
// #include <monopod_sdk/monopod.hpp>
// This is not imported because it should not be a connected graph for imports.
// Monopod in the parent to leg and planarizer. this means it should not be imported lower down... that should be
// invariant.


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

    Planarizer::Planarizer(
            std::shared_ptr<monopod_drivers::EncoderInterface> encoder_by,
            std::shared_ptr<monopod_drivers::EncoderInterface> encoder_bp,
            std::shared_ptr<monopod_drivers::EncoderInterface> encoder_bc)
    {
        encoders_[boom_connector] = encoder_bc;
        encoders_[boom_yaw] = encoder_by;
        encoders_[boom_pitch] = encoder_bp;
    }

    Ptr<const ScalarTimeseries> Planarizer::get_measurement(const int &joint_index,
                                        const int &measurement_index) const
    {
        return encoders_[joint_index]->get_measurement(measurement_index);
    }

} // end monopod_drivers namespace
