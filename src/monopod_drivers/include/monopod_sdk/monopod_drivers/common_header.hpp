#pragma once
#include <time_series/time_series.hpp>

namespace monopod_drivers
{

    // Planarizer matrices for storing data ==================================

    /**
     * @brief Defines a dynamic sized Eigen vector type to hold the encoders.
     * May have two or three encoders for optional fixed/free hip
     */
    typedef Eigen::Matrix<double, Eigen::Dynamic, 1> PVector;

    /**
     * @brief Defines a dynamic sized Eigen matrix type to store all the data
     * from the encoders. The two columns correspond to pos and vel.
     */
    typedef Eigen::Matrix<double, Eigen::Dynamic, 2> PMatrix;


    // Leg matrices for storing data ==================================

    /**
     * @brief Defines a static sized Eigen vector type to store data for the leg.
     * Data is one of pos, vel, torque
     */
    typedef Eigen::Matrix<double, 2, 1> LVector;

    /**
     * @brief Defines a static sized Eigen vector type to store all the data for the leg.
     * Data columns are pos, vel, torque
     */
    typedef Eigen::Matrix<double, 2, 3> LMatrix;


    /**
     * @brief This is a useful alias.
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

    // ====================================================================================

    /**
    * @brief Enumerates the joint names for indexing
    */
    enum JointNameIndexing
    {
      hip_joint,
      knee_joint,
      boom_connector_joint,
      planarizer_yaw_joint,
      planarizer_pitch_joint
    };

} // end namespace monopod_drivers
