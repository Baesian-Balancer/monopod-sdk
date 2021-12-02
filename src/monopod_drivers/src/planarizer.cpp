#include "planarizer.hpp"

namespace monopod_drivers
{

    Planarizer(std::shared_ptr<monopod_drivers::EncoderInterface> encoder_bc,
            std::shared_ptr<monoopod_drivers::EncoderInterface> encoder_by,
            std::shared_ptr<monopod_drivers::EncoderInterface> encoder_bp)
    {
        encoders_[boom_connector] = encoder_bc;
        encoders_[boom_yaw] = encoder_by;
        encoders_[boom_pitch] = encoder_bp;
    }

    ReturnValueStatus get_measurement(const int &joint_index,
                                        const int &measurement_index)
    {
        return encoders_[joint_index]->get_measurement(measurement_index)
    }

} // end monopod_drivers namespace