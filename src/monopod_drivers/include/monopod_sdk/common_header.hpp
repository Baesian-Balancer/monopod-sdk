#pragma once
#include <time_series/time_series.hpp>

namespace monopod_drivers
{

    // Planarizer matrices for storing data ==================================

    // Leg matrices for storing data ==================================

    /**
     * @brief Defines a static sized Eigen vector type to store data for the leg.
     * Data is one of pos, vel, torque
     */
    typedef Eigen::Matrix<double, 2, 1> LVector;


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


    /**
     * @brief Here is a list of the different measurement available on the
     * blmc card.
     */
     enum MeasurementIndex
     {
         position,
         velocity,
         acceleration,
         current,
         encoder_index,
         measurement_count, //Meassurement count is the 'length' of the meassurement vector in motor board.
         torque,
     };

    /**
    * @brief Joint names indexed same as enumerator
    */
    const std::unordered_map<JointNameIndexing, int> JointModulesIndexMapping = {
        {hip_joint, 0},
        {knee_joint, 1},
        {boom_connector_joint, 2},
        {planarizer_yaw_joint, 1},
        {planarizer_pitch_joint, 0}
    };

} // end namespace monopod_drivers
