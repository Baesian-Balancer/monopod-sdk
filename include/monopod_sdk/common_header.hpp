#pragma once
#include <time_series/time_series.hpp>

namespace monopod_drivers {

/**
 * ================================================
 * Definitions of constants for robot
 * ================================================
 */

/**
 * Max current before the robot gets kill in amps (A).
 */
#define MAX_CURRENT 20.0

/**
 * Number of joints in a monopod_drivers::Leg.
 */
#define NUMBER_LEG_JOINTS 2

/**
 * ================================================
 * Type defs
 * ================================================
 */

/**
 * @brief A useful shortcut
 */
typedef time_series::TimeSeries<double> ScalarTimeseries;
/**
 * @brief A useful shortcut
 */
typedef time_series::Index Index;
/**
 * @brief A useful shortcut
 */
typedef time_series::TimeSeries<Index> IndexTimeseries;

/**
 * @brief This is a shortcut for creating shared pointer in a simpler
 * writing expression.
 *
 * @tparam Type is the template parameter of the shared pointer.
 */
template <typename Type> using Ptr = std::shared_ptr<Type>;

/**
 * @brief A useful shortcut
 */
template <typename Type> using Vector = std::vector<Type>;

// ====================================================================================

/**
 * @brief Enumerates the joint names for indexing
 */
enum JointNamesIndex {
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
enum Measurements {
  position,
  velocity,
  acceleration,
  current,
  encoder_index,
  measurement_count, // Meassurement count is the 'length' of the meassurement
                     // vector in motor board.
};

/**
 * @brief Mode defines the boards (joints) that are being considered from the
 * canbus. This allows us to handle say only wanting to use the leg or only
 * wanting to read plnarizer sensors. Also defines diferent task modes for the
 * gym environment.
 */
enum class Mode {
  /**
   * @brief Complete free boom connector (5 joints total)
   */
  Free,

  /**
   * @brief Fixed boom connector (4 joints total)
   */
  Fixed_connector,

  /**
   * @brief Fixed boom connector and planrizer yaw (3 joints total)
   */
  Fixed,

  /**
   * @brief motor board
   */
  motor_board,

  /**
   * @brief encoder board 1
   */
  encoder_board1,

  /**
   * @brief encoder board 2
   */
  encoder_board2
};

/**
 * @brief Structure holding the PID values for the joint.
 */
struct PID {
  PID() = default;
  PID(const double _p, const double _i, const double _d)
      : p(_p), i(_i), d(_d) {}

  double p = 0;
  double i = 0;
  double d = 0;
};
/**
 * @brief Structure holding joint limits
 */
struct JointLimit {
  JointLimit() {
    constexpr double m = std::numeric_limits<double>::lowest();
    constexpr double M = std::numeric_limits<double>::max();

    min = m;
    max = M;
  }

  JointLimit(const double _min, const double _max) : min(_min), max(_max) {}

  double min;
  double max;
};

} // end namespace monopod_drivers
