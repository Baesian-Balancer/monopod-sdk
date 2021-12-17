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

std::vector<std::string> Monopod::get_jointNames() const
{
    return joint_names;
}

bool Monopod::set_torque_target(const double &torque_target, const int joint_index)
{
    switch(joint_index)
    {
        case hip_joint:
        case knee_joint:
            buffers.write_door.lock(); //Lock write buffers
            buffers.write[(JointNameIndexing)joint_index] = torque_target;
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
    buffers.write_door.lock(); //Lock write buffers
    for(std::vector<int>::size_type i = 0; i != torque_targets.size(); i++){
        switch(jointSerialization[i])
        {
            case hip_joint:
            case knee_joint:
                buffers.write[(JointNameIndexing)jointSerialization[i]] = torque_targets[i];
                ok = ok &&  true;
                break;
            default:
                ok = ok &&  false;
                break;
        }
    }
    buffers.write_door.unlock(); //unLock write buffers

    return ok;
}

std::optional<double> Monopod::get_torque_target(const int &joint_index)
{
    switch(joint_index)
    {
        case hip_joint:
        case knee_joint:
        {  
              buffers.write_door.lock(); //Lock write buffers
              double torque_target = buffers.write[(JointNameIndexing)joint_index];
              buffers.write_door.unlock(); //unLock write buffers
              return torque_target;
        }
        default:
            return std::nullopt;

    }
}

std::optional<std::vector<double>> Monopod::get_torque_targets(const std::vector<int> &joint_indexes)
{
    const std::vector<int>& jointSerialization =
        joint_indexes.empty() ? motor_joint_indexing : joint_indexes;

    std::vector<double> data;
    data.reserve(jointSerialization.size());
    buffers.write_door.lock(); //Lock write buffers
    for(auto& joint_index : jointSerialization){
        switch(joint_index)
        {
            case hip_joint:
            case knee_joint:
                data.push_back(buffers.write[(JointNameIndexing)joint_index]);
                break;
            default:
                buffers.write_door.unlock();
                return std::nullopt;
        }
    }
    buffers.write_door.unlock(); //unLock write buffers
    return data;
}

std::optional<double> Monopod::get_position(const int &joint_index)
{
  switch(joint_index)
  {
      case hip_joint:
      case knee_joint:
      case boom_connector_joint:
      case planarizer_yaw_joint:
      case planarizer_pitch_joint:
      {
          buffers.read_door.lock();
          double position = buffers.read[(JointNameIndexing)joint_index].pos;
          buffers.read_door.unlock();
          return position;
      }
      default:
          return std::nullopt;

  }
}


std::optional<double> Monopod::get_velocity(const int &joint_index)
{
  switch(joint_index)
  {
      case hip_joint:
      case knee_joint:
      case boom_connector_joint:
      case planarizer_yaw_joint:
      case planarizer_pitch_joint:
      {
          buffers.read_door.lock();
          double velocity = buffers.read[(JointNameIndexing)joint_index].vel;
          buffers.read_door.unlock();
          return velocity;
      }
      default:
          return std::nullopt;

  }
}


std::optional<double> Monopod::get_acceleration(const int &joint_index)
{
  switch(joint_index)
  {
      case hip_joint:
      case knee_joint:
      case boom_connector_joint:
      case planarizer_yaw_joint:
      case planarizer_pitch_joint:
      {
          buffers.read_door.lock();
          double acceleration = buffers.read[(JointNameIndexing)joint_index].acc;
          buffers.read_door.unlock();
          return acceleration;
      }
      default:
          return std::nullopt;

  }
}

std::optional<std::vector<double>> Monopod::get_positions(const std::vector<int> &joint_indexes)
{
  auto lambda = [](JointReadState joint_state) -> double  {
      return joint_state.pos;
  };

  return getJointDataSerialized(this, joint_indexes, lambda);
}

std::optional<std::vector<double>> Monopod::get_velocities(const std::vector<int> &joint_indexes)
{
  auto lambda = [](JointReadState joint_state) -> double  {
      return joint_state.vel;
  };

  return getJointDataSerialized(this, joint_indexes, lambda);
}

std::optional<std::vector<double>> Monopod::get_accelerations(const std::vector<int> &joint_indexes)
{
  auto lambda = [](JointReadState joint_state) -> double  {
      return joint_state.acc;
  };

  return getJointDataSerialized(this, joint_indexes, lambda);
}

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
  size_t count_in = 0;
  while (!stop_loop)
  {

      // Do stuff --------------------------------------------------------



      // print -----------------------------------------------------------

      if ((count % 2500) == 0)
      {
          rt_printf("Loop number: %ld\n", count);
          // Write buffers print ------------------------------------------

          // buffers.write_door.lock(); //Lock write buffers
          //
          // rt_printf("buffers.write: ");
          // for (auto const &pair: buffers.write) {
          //     // std::cout << "{" << pair.first << ": " << pair.second << "}";
          //     rt_printf("{key: %s, Val: %f}", joint_names[pair.first].c_str(), pair.second);
          // }
          // rt_printf("\n");
          //
          // buffers.write_door.unlock(); //unLock write buffers

          // Read buffers print -------------------------------------------

          buffers.read_door.lock(); //Lock read buffers

          // rt_printf("buffers.read: ");
          // for (auto const &pair: buffers.read) {
          //     // std::cout << "{" << pair.first << ": " << pair.second << "}";
          //     rt_printf("{key: %s, Val: %f}", joint_names[pair.first].c_str(), pair.second.pos);
          // }
          // rt_printf("\n");

          buffers.read[(JointNameIndexing)(count_in%5)].pos++;
          buffers.read[(JointNameIndexing)(count_in%5)].vel++;
          buffers.read[(JointNameIndexing)(count_in%5)].acc++;

          buffers.read_door.unlock(); //unLock read buffers
          count_in++;
      }
      count++;

      spinner.spin();
  }
}

} // end monopod_drivers namespace
