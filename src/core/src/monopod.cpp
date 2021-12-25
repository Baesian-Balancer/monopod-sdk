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

bool Monopod::is_joint_controllable(const int joint_index)
{
  switch(joint_index)
  {
      case hip_joint:
      case knee_joint:
      {
          return true;
      }
      default:
          return false;

  }
}

// ========================================
// Getters
// ========================================

std::string Monopod::get_model_name() const {
    return "monopod";
}

std::unordered_map<std::string, int> Monopod::get_joint_names() const
{
    return joint_names;
}

std::optional<Monopod::PID> Monopod::get_pid(const int &joint_index)
{
    switch(joint_index)
    {
        case hip_joint:
        case knee_joint:
        {
              buffers.pid_door.lock(); //Lock pid buffers
                const PID _pid = buffers.pid[(JointNameIndexing)joint_index];
              buffers.pid_door.unlock(); //unLock pid buffers
              return _pid;
        }
        default:
            return std::nullopt;

    }
}

std::optional<Monopod::JointLimit> Monopod::get_joint_position_limit(const int &joint_index)
{
  switch(joint_index)
  {
      case hip_joint:
      case knee_joint:
      case boom_connector_joint:
      case planarizer_yaw_joint:
      case planarizer_pitch_joint:
      {
          buffers.settings_door.lock();
            const Monopod::JointLimit limit = buffers.settings[(JointNameIndexing)joint_index].position_limit;
          buffers.settings_door.unlock();
          return limit;
      }
      default:
          return std::nullopt;

  }
}

std::optional<Monopod::JointLimit> Monopod::get_joint_velocity_limit(const int &joint_index)
{
  switch(joint_index)
  {
      case hip_joint:
      case knee_joint:
      case boom_connector_joint:
      case planarizer_yaw_joint:
      case planarizer_pitch_joint:
      {
          buffers.settings_door.lock();
            const Monopod::JointLimit limit = buffers.settings[(JointNameIndexing)joint_index].velocity_limit;
          buffers.settings_door.unlock();
          return limit;
      }
      default:
          return std::nullopt;

  }
}

std::optional<Monopod::JointLimit> Monopod::get_joint_acceleration_limit(const int &joint_index)
{
  switch(joint_index)
  {
      case hip_joint:
      case knee_joint:
      case boom_connector_joint:
      case planarizer_yaw_joint:
      case planarizer_pitch_joint:
      {
          buffers.settings_door.lock();
            const Monopod::JointLimit limit = buffers.settings[(JointNameIndexing)joint_index].acceleration_limit;
          buffers.settings_door.unlock();
          return limit;
      }
      default:
          return std::nullopt;

  }
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
    auto lambda = [this](int joint_index) -> double  {
        return buffers.read[(JointNameIndexing)joint_index].pos;
    };

    buffers.read_door.lock();
      auto data = getJointDataSerialized(this, joint_indexes, lambda);
    buffers.read_door.unlock();
    return data;
}

std::optional<std::vector<double>> Monopod::get_velocities(const std::vector<int> &joint_indexes)
{
    auto lambda = [this](int joint_index) -> double  {
        return buffers.read[(JointNameIndexing)joint_index].vel;
    };

    buffers.read_door.lock();
      auto data = getJointDataSerialized(this, joint_indexes, lambda);
    buffers.read_door.unlock();
    return data;
}

std::optional<std::vector<double>> Monopod::get_accelerations(const std::vector<int> &joint_indexes)
{
    auto lambda = [this](int joint_index) -> double  {
        return buffers.read[(JointNameIndexing)joint_index].acc;
    };

    buffers.read_door.lock();
      auto data = getJointDataSerialized(this, joint_indexes, lambda);
    buffers.read_door.unlock();
    return data;
}

std::optional<double> Monopod::get_max_torque_target(const int &joint_index)
{
    switch(joint_index)
    {
        case hip_joint:
        case knee_joint:
        case boom_connector_joint:
        case planarizer_yaw_joint:
        case planarizer_pitch_joint:
        {
            buffers.settings_door.lock();
              double max_target = buffers.settings[(JointNameIndexing)joint_index].max_torque_target;
            buffers.settings_door.unlock();
            return max_target;
        }
        default:
            return std::nullopt;
    }
}

// ========================================
// Setters
// ========================================

bool Monopod::set_pid(const int &p, const int &i, const int &d, const int &joint_index)
{
    switch(joint_index)
    {
        case hip_joint:
        case knee_joint:
            buffers.pid_door.lock(); //Lock pid buffers
              buffers.pid[(JointNameIndexing)joint_index].p = p;
              buffers.pid[(JointNameIndexing)joint_index].i = i;
              buffers.pid[(JointNameIndexing)joint_index].d = d;
            buffers.pid_door.unlock(); //unLock pid buffers
            return true;
        default:
            return false;

    }
}

bool Monopod::set_joint_position_limit(const double& max, const double& min, const int &joint_index)
{
    switch(joint_index)
    {
        case hip_joint:
        case knee_joint:
        case boom_connector_joint:
        case planarizer_yaw_joint:
        case planarizer_pitch_joint:
        {
            buffers.settings_door.lock();
              buffers.settings[(JointNameIndexing)joint_index].position_limit.max = max;
              buffers.settings[(JointNameIndexing)joint_index].position_limit.min = min;
            buffers.settings_door.unlock();
            return true;
        }
        default:
            return false;

    }
}

bool Monopod::set_joint_velocity_limit(const double& max, const double& min, const int &joint_index)
{
    switch(joint_index)
    {
        case hip_joint:
        case knee_joint:
        case boom_connector_joint:
        case planarizer_yaw_joint:
        case planarizer_pitch_joint:
        {
            buffers.settings_door.lock();
              buffers.settings[(JointNameIndexing)joint_index].velocity_limit.max = max;
              buffers.settings[(JointNameIndexing)joint_index].velocity_limit.min = min;
            buffers.settings_door.unlock();
            return true;
        }
        default:
            return false;

    }
}

bool Monopod::set_joint_acceleration_limit(const double& max, const double& min, const int &joint_index)
{
    switch(joint_index)
    {
        case hip_joint:
        case knee_joint:
        case boom_connector_joint:
        case planarizer_yaw_joint:
        case planarizer_pitch_joint:
        {
            buffers.settings_door.lock();
              buffers.settings[(JointNameIndexing)joint_index].acceleration_limit.max = max;
              buffers.settings[(JointNameIndexing)joint_index].acceleration_limit.min = min;
            buffers.settings_door.unlock();
            return true;
        }
        default:
            return false;

    }
}

bool Monopod::set_max_torque_target(const double &max_torque_target,  const int &joint_index)
{
    switch(joint_index)
    {
        case hip_joint:
        case knee_joint:
        case boom_connector_joint:
        case planarizer_yaw_joint:
        case planarizer_pitch_joint:
        {
            buffers.settings_door.lock();
              // printf("new max torque: %f\n", max_torque_target );
              buffers.settings[(JointNameIndexing)joint_index].max_torque_target = max_torque_target;
            buffers.settings_door.unlock();
            return true;
        }
        default:
            return false;
    }
}

bool Monopod::set_torque_target(const double &torque_target, const int joint_index)
{
    switch(joint_index)
    {
        case hip_joint:
        case knee_joint:
        {
            buffers.settings_door.lock(); //Lock settings buffers
              double max_torque_target = buffers.settings[(JointNameIndexing)joint_index].max_torque_target;
            buffers.settings_door.unlock(); //unLock settings buffers

            buffers.write_door.lock(); //Lock write buffers
              // Clip to max if over.
              if(std::abs(torque_target) > max_torque_target)
              {
                  int force_dir = Monopod::sgn(torque_target);
                  buffers.write[(JointNameIndexing)joint_index] = force_dir * max_torque_target;
              }else{
                buffers.write[(JointNameIndexing)joint_index] = torque_target;
              }
            buffers.write_door.unlock(); //unLock write buffers

            return true;
        }
        default:
            return false;

    }
}


bool Monopod::set_torque_targets(const std::vector<double> &torque_targets, const std::vector<int> &joint_indexes)
{

     // Note: if it fails the behaviour is undefined. For example if first 3 joints
     // are right but one bad index it will updatethe good ones the fail on the bad one
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

      // Get data from board

      // Check Limits

      // Set Torque


      // Random tests ----------------------------------------------------

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
