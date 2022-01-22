#pragma once
#include <time_series/time_series.hpp>

namespace monopod_drivers {

/** Definitions of constants for robot
 */

/** Max current before the robot gets kill in amps (A).
 */
#define MAX_CURRENT 20.0
#define NUMBER_LEG_JOINTS 2

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
template <typename Type> using Ptr = std::shared_ptr<Type>;

// ====================================================================================

/**
 * @brief Enumerates the joint names for indexing
 */
enum JointNameIndexing {
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
enum MeasurementIndex {
  position,
  velocity,
  acceleration,
  current,
  encoder_index,
  measurement_count, // Meassurement count is the 'length' of the meassurement
                     // vector in motor board.
  torque             // this is only used for monopodsdk
};

typedef std::unordered_map<MeasurementIndex, double> map_inner;
typedef std::unordered_map<JointNameIndexing, map_inner> ObservationMap;
typedef std::unordered_map<JointNameIndexing, double> ActionMap;

enum class Mode {
  /**
   * @brief Complete free boom connector (5 joints total)
   *
   */
  Free,

  /**
   * @brief Fixed boom connector (4 joints total)
   *
   */
  Fixed_connector,

  /**
   * @brief Fixed boom connector and planrizer yaw (3 joints total)
   *
   */
  Fixed,

  /**
   * @brief Specify custom joints to initialize
   *
   */
  Custom
};

} // end namespace monopod_drivers
