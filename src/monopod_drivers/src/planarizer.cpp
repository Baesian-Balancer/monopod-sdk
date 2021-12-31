
#include "monopod_sdk/monopod_drivers/planarizer.hpp"

namespace monopod_drivers
{


    /**
     * @brief Defines a static Eigen vector type in order to define the
     * interface. Three or two encoders
     */
    typedef Eigen::Matrix<double, Eigen::Dynamic, 1> Vector;

    /**
     * @brief Defines a static Eigen matrix type in order to define the
     * interface. Three rows for encoders, 2 columns for pos and vel.
     */
    typedef Eigen::Matrix<double, Eigen::Dynamic, 2> Matrix;
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
        encoders_.push_back(encoder_by);
        encoders_.push_back(encoder_bp);
        encoders_.push_back(encoder_bc);

        fixed_ = false;

        // encoders_[boom_yaw] = encoder_by;
        // encoders_[boom_pitch] = encoder_bp;
        // encoders_[boom_connector] = encoder_bc;
    }

    Planarizer::Planarizer(
            std::shared_ptr<monopod_drivers::EncoderInterface> encoder_by,
            std::shared_ptr<monopod_drivers::EncoderInterface> encoder_bp)
    {
        encoders_.push_back(encoder_by);
        encoders_.push_back(encoder_bp);

        fixed_ = true;
    }

    Vector Planarizer::get_measurements(const int &measurement_index) const
    {
        Vector data;
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
            printf("Wrong measurement index passed. Must be position or velocity")
        }

        return data;
    }

    Matrix Planarizer::get_data()
    {
        Matrix all_data;

        for(int pidx = boom_yaw; pidx != PI_end; pidx++)
        {
            if(fixed_ && pidx == boom_connector){continue;}

            all_data(pidx, position) = encoders_[pidx]->get_measurement(position)->newest_element();
            all_data(pidx, velocity) = encoders_[pidx]->get_measurement(velocity)->newest_element();
        }
        return all_data;
    }

} // end monopod_drivers namespace
