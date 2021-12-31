
#pragma once

#include <time_series/time_series.hpp>
#include "monopod_sdk/monopod_drivers/encoder.hpp"

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
         * @brief Defines a static Eigen vector type in order to define the
         * interface. Three for number of encoders
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

        // struct ReturnValueStatus
        // {
        //     bool valid;
        //     Ptr<ScalarTimeseries> value_series;
        // };

        enum PlanarizerIndexing
        {
            boom_yaw,
            boom_pitch,
            boom_connector,
            PI_end
        };

        enum MeasurementIndexing
        {
            position,
            velocity,
        };

        /**
         * @brief Construct a new Planarizer object with all 3 encoders
         *
         * @param encoder_bc  boom connector encoder
         * @param encoder_by  boom yaw encoder
         * @param encoder_bp  boom pitch encoder
         */
        Planarizer(std::shared_ptr<EncoderInterface> encoder_by,
                    std::shared_ptr<EncoderInterface> encoder_bp,
                    std::shared_ptr<EncoderInterface> encoder_bc);

        /**
         * @brief Construct a new Planarizer object with 2 encoders;
         * use when boom connector is fixed
         * 
         * @param encoder_by boom yaw encoder
         * @param encoder_bp boom pitch encoder
         */
        Planarizer(std::shared_ptr<EncoderInterface> encoder_by,
                    std::shared_ptr<EncoderInterface> encoder_bp);

        /**
         * @brief Destroy the Planarizer object
         */
        ~Planarizer(){};

        /**
         * Getters
         */

        /**
         * @brief Get the specified measurement (pos or vel) for all joints
         * 
         * @param measurement_index 
         * @return Vector 
         */
        Vector get_measurements(const int &measurement_index) const;

        /**
         * @brief Return all data (pos and vel) for all joints
         * 
         * @return Matrix 
         */
        Matrix get_data();

    private:

        /**
         * @brief Vector to hold the encoders of the planarizer
         */
        std::vector<std::shared_ptr<EncoderInterface>> encoders_;

        /**
         * @brief Flag to indicate if using fixed boom connector
         */
        bool fixed_;


};
} // end monopod_drivers namespace
