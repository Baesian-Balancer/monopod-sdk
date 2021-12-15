#include "monopod_sdk/monopod.hpp"


namespace monopod_drivers
{

  /**
   * @brief this is a simple loop which runs at a kilohertz.
   *
   * Prints every 1k iterations
   */
  void Monopod::loop()
  {
      real_time_tools::Spinner spinner;
      spinner.set_period(0.001);  // 1kz loop
      size_t count = 0;
      while (!stop_loop_)
      {

          // Do stuff --------------------------------------------------------

          // print -----------------------------------------------------------
          spinner.spin();
          if ((count % 1000) == 0)
          {
              rt_printf("Loop number: %d\n", count);
          }
          count++;
      }
  }

} // end monopod_drivers namespace
