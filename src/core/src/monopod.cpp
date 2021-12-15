#include "monopod_sdk/monopod.hpp"

namespace monopod_drivers
{

//===================================================================
// Public methods
//===================================================================

Monopod::Monopod()
{
    stop_loop = false;
}

Monopod::~Monopod()
{
    stop_loop = true;
    rt_thread_.join();
}

bool Monopod::initialize()
{
    return true;
}

void Monopod::start_loop()
{
    rt_thread_.create_realtime_thread(&Monopod::loop, this);
}

bool Monopod::set_torque_target(const double &torque_target, const int joint_index)
{
    switch(joint_index)
    {
        case hip_joint:
            buffers.write_door.lock(); //Lock write buffers
            buffers.write[hip_joint] = torque_target;
            buffers.write_door.unlock(); //unLock write buffers
            return true;
        case knee_joint:
            buffers.write_door.lock(); //Lock write buffers
            buffers.write[knee_joint] = torque_target;
            buffers.write_door.unlock(); //unLock write buffers
            return true;
        default:
            return false;

    }
}

bool Monopod::set_torque_targets(const std::vector<double> &torque_targets, const std::vector<int> &joint_indexes)
{

    const std::vector<int>& jointSerialization =
        joint_indexes.empty() ? motor_joint_indexing : joint_indexes;

    if(torque_targets.size() != jointSerialization.size())
        // This means the inputs do not match
        return false;

    bool ok = true;
    for(std::vector<int>::size_type i = 0; i != torque_targets.size(); i++){
    // for (const auto& joint_index : jointSerialization) {
        switch(jointSerialization[i])
        {
            case hip_joint:
                buffers.write_door.lock(); //Lock write buffers
                buffers.write[hip_joint] = torque_targets[i];
                buffers.write_door.unlock(); //unLock write buffers
                ok = ok &&  true;
            case knee_joint:
                buffers.write_door.lock(); //Lock write buffers
                buffers.write[knee_joint] = torque_targets[i];
                buffers.write_door.unlock(); //unLock write buffers
                ok = ok &&  true;
            default:
                ok = ok &&  false;

        }
    }

    return ok;
}

// std::optional<double> Monopod::get_measurements()
// {
//     double value = 0.0;
//
//     switch(joint_index)
//     {
//         case hip_joint:
//             return value;
//         case knee_joint:
//             return value;
//         default:
//             return std::nullopt;
//
//     }
// }


//===================================================================
// Private methods
//===================================================================

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
  while (!stop_loop)
  {

      // Do stuff --------------------------------------------------------



      // print -----------------------------------------------------------

      if ((count % 2500) == 0)
      {
          rt_printf("Loop number: %ld\n", count);

          // Write buffers print ------------------------------------------

          buffers.write_door.lock(); //Lock write buffers

          rt_printf("buffers.write: ");
          for (auto const &pair: buffers.write) {
              // std::cout << "{" << pair.first << ": " << pair.second << "}";
              rt_printf("{key: %s, Val: %f}", joint_names[pair.first].c_str(), pair.second);
          }
          rt_printf("\n");

          buffers.write_door.unlock(); //unLock write buffers

          // // Read buffers print -------------------------------------------
          //
          // buffers.read_door.lock(); //Lock read buffers
          //
          // rt_printf("buffers.read: ");
          // for (auto const &pair: buffers.read) {
          //     // std::cout << "{" << pair.first << ": " << pair.second << "}";
          //     rt_printf("{key: %s, Val: %f}", pair.first.c_str(), pair.second);
          // }
          // rt_printf("\n");
          //
          // buffers.read_door.unlock(); //unLock read buffers
      }
      count++;

      spinner.spin();
  }
}

} // end monopod_drivers namespace
