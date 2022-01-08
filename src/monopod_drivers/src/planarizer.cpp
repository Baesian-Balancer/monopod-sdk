
#include "monopod_sdk/monopod_drivers/planarizer.hpp"

namespace monopod_drivers
{
    Planarizer::Planarizer(
            Ptr<monopod_drivers::EncoderInterface> encoder_by,
            Ptr<monopod_drivers::EncoderInterface> encoder_bp,
            Ptr<monopod_drivers::EncoderInterface> encoder_bc)
    {
        encoders_.push_back(encoder_by);
        encoders_.push_back(encoder_bp);
        encoders_.push_back(encoder_bc);

        fixed_ = false;
    }

    Planarizer::Planarizer(
            Ptr<monopod_drivers::EncoderInterface> encoder_by,
            Ptr<monopod_drivers::EncoderInterface> encoder_bp)
    {
        encoders_.push_back(encoder_by);
        encoders_.push_back(encoder_bp);

        fixed_ = true;
    }

    PVector Planarizer::get_measurements(const int &measurement_index) const
    {
        PVector data;
        if(measurement_index == position || measurement_index == velocity)
        {
            if(fixed_)
            {
                data[0] = encoders_[boom_yaw]->get_measurement(measurement_index)->newest_element();
                data[1] = encoders_[boom_pitch]->get_measurement(measurement_index)->newest_element();
            }
            else
            {
                data[0] = encoders_[boom_yaw]->get_measurement(measurement_index)->newest_element();
                data[1] = encoders_[boom_pitch]->get_measurement(measurement_index)->newest_element();
                data[2] = encoders_[boom_connector]->get_measurement(measurement_index)->newest_element();
            }
        }
        else
        {
            printf("Wrong measurement index passed. Must be position or velocity");
        }

        return data;
    }

    PMatrix Planarizer::get_data()
    {
        PMatrix all_data;

        for(int pidx = boom_yaw; pidx != PI_end; pidx++)
        {
            if(fixed_ && pidx == boom_connector){continue;}

            all_data(pidx, position - 1) = encoders_[pidx]->get_measurement(position)->newest_element();
            all_data(pidx, velocity - 1) = encoders_[pidx]->get_measurement(velocity)->newest_element();
        }
        return all_data;
    }

} // end monopod_drivers namespace
