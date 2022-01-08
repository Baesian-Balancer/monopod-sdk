
#pragma once

#include <time_series/time_series.hpp>
#include "monopod_sdk/monopod_drivers/common_header.hpp"

#include "monopod_sdk/monopod_drivers/encoder.hpp"

namespace monopod_drivers
{
/**
 * @brief This class defines an interface to the planarizer, which consists of
 * the encoders for boom yaw, boom pitch, and boom-connector angle (free angle between
 * the hip piece and boom)
 * 
 */
class Planarizer
{
    public:
        /**
         * @brief Enumerate the joints of the planarizer
         * 
         */
        enum PlanarizerIndexing
        {
            boom_yaw,
            boom_pitch,
            boom_connector,
            PI_end
        };

        /**
         * @brief Enumerate measurement indices
         * 
         */
        enum MeasurementIndexing
        {
            position = 1,
            velocity = 2,
        };

        /**
         * @brief Construct a new Planarizer object with all 3 encoders
         *
         * @param encoder_bc  boom connector encoder
         * @param encoder_by  boom yaw encoder
         * @param encoder_bp  boom pitch encoder
         */
        Planarizer(Ptr<EncoderInterface> encoder_by,
                    Ptr<EncoderInterface> encoder_bp,
                    Ptr<EncoderInterface> encoder_bc);

        /**
         * @brief Construct a new Planarizer object with 2 encoders;
         * use when boom connector is fixed
         * 
         * @param encoder_by boom yaw encoder
         * @param encoder_bp boom pitch encoder
         */
        Planarizer(Ptr<EncoderInterface> encoder_by,
                    Ptr<EncoderInterface> encoder_bp);

        /**
         * @brief Destroy the Planarizer object
         */
        ~Planarizer(){};


        /// getters ================================================================

        /**
         * @brief Get the specified measurement (pos or vel) for all joints
         * 
         * @param measurement_index 
         * @return PVector 
         */
        PVector get_measurements(const int &measurement_index) const;

        /**
         * @brief Return all data (pos and vel) for all joints
         * 
         * @return PMatrix 
         */
        PMatrix get_data();

    private:

        /**
         * @brief Vector to hold the encoders of the planarizer
         */
        std::vector<Ptr<EncoderInterface>> encoders_;

        /**
         * @brief Flag to indicate if using fixed boom connector
         */
        bool fixed_;


};
} // end monopod_drivers namespace
