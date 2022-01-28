#pragma once
#include <time_series/time_series.hpp>

namespace monopod_drivers {

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

} // end namespace monopod_drivers
