#include "monopod_sdk/monopod.hpp"
#include <stdexcept>

namespace monopod_drivers
{

//===================================================================
// Public methods
//===================================================================

Monopod::Monopod()
{
    stop_loop = false;
    read_joint_indexing =
     {
       hip_joint,
       knee_joint,
       boom_connector_joint,
       planarizer_yaw_joint,
       planarizer_pitch_joint
     };
     write_joint_indexing =
     {
       hip_joint,
       knee_joint
     };
     can_bus_ = std::make_shared<monopod_drivers::CanBus>("can0");
     board_ = std::make_shared<monopod_drivers::CanBusMotorBoard>(can_bus_);

     leg_ = std::make_unique<monopod_drivers::Leg>(board_);
     planarizer_ = std::make_unique<monopod_drivers::Planarizer>(board_, 2);


}

Monopod::~Monopod()
{
    stop_loop = true;
    rt_thread_.join();
}

bool Monopod::initialize()
{

    leg_->initialize();
    planarizer_->initialize();

    is_initialized = true;
    return initialized();
}

bool Monopod::initialized()
{
    return is_initialized;
}

void Monopod::start_loop()
{
    if (is_initialized)
    {
      rt_thread_.create_realtime_thread(&Monopod::loop, this);
    }
    else
    {
      throw std::runtime_error("Need to initialize monopod_sdk before starting the realtime loop.");
    }
}

bool Monopod::is_joint_controllable(const int joint_index)
{
    return is_initialized && Contains(write_joint_indexing, joint_index);
}

// ========================================
// Getters
// ========================================

std::string Monopod::get_model_name() const {
    return "monopod";
}

std::unordered_map<std::string, int> Monopod::get_joint_names() const
{
    std::unordered_map<std::string, int> joint_names_;

    for (auto const &pair: joint_names) {
        if (Contains(read_joint_indexing, pair.second) || Contains(write_joint_indexing, pair.second)){
            joint_names_[pair.first] = pair.second;
        }
    }
    return joint_names_;
}

std::optional<Monopod::PID> Monopod::get_pid(const int &joint_index)
{
    if (is_initialized && Contains(write_joint_indexing, joint_index))
    {
          buffers.pid_door.lock(); //Lock pid buffers
            const PID _pid = buffers.pid[(JointNameIndexing)joint_index];
          buffers.pid_door.unlock(); //Unlock pid buffers
          return _pid;
    }
    return std::nullopt;

}

std::optional<Monopod::JointLimit> Monopod::get_joint_position_limit(const int &joint_index)
{
    if (is_initialized && Contains(read_joint_indexing, joint_index))
    {
        buffers.settings_door.lock();
          const Monopod::JointLimit limit = buffers.settings[(JointNameIndexing)joint_index].position_limit;
        buffers.settings_door.unlock();
        return limit;
    }
    return std::nullopt;
}

std::optional<Monopod::JointLimit> Monopod::get_joint_velocity_limit(const int &joint_index)
{
    if (is_initialized && Contains(read_joint_indexing, joint_index))
    {
        buffers.settings_door.lock();
          const Monopod::JointLimit limit = buffers.settings[(JointNameIndexing)joint_index].velocity_limit;
        buffers.settings_door.unlock();
        return limit;
    }
    return std::nullopt;
}

std::optional<Monopod::JointLimit> Monopod::get_joint_acceleration_limit(const int &joint_index)
{
    if (is_initialized && Contains(read_joint_indexing, joint_index))
    {
        buffers.settings_door.lock();
          const Monopod::JointLimit limit = buffers.settings[(JointNameIndexing)joint_index].acceleration_limit;
        buffers.settings_door.unlock();
        return limit;
    }
    return std::nullopt;
}

std::optional<double> Monopod::get_torque_target(const int &joint_index)
{
    if (is_initialized && Contains(write_joint_indexing, joint_index))
    {
          buffers.write_door.lock(); //Lock write buffers
            double torque_target = buffers.write[(JointNameIndexing)joint_index];
          buffers.write_door.unlock(); //Unlock write buffers
          return torque_target;
    }
    return std::nullopt;
}

std::optional<std::vector<double>> Monopod::get_torque_targets(const std::vector<int> &joint_indexes)
{
    const std::vector<int>& jointSerialization =
        joint_indexes.empty() ? write_joint_indexing : joint_indexes;

    std::vector<double> data;
    data.reserve(jointSerialization.size());
    buffers.write_door.lock(); //Lock write buffers
      for(auto& joint_index : jointSerialization){
          if (is_initialized && Contains(write_joint_indexing, joint_index))
          {
              data.push_back(buffers.write[(JointNameIndexing)joint_index]);
              continue;
          }
          else
          {
              buffers.write_door.unlock();
              return std::nullopt;
          }
      }
    buffers.write_door.unlock(); //Unlock write buffers
    return data;

}

std::optional<double> Monopod::get_position(const int &joint_index)
{
    if (is_initialized && Contains(read_joint_indexing, joint_index))
    {
        buffers.read_door.lock();
          double pos = buffers.read[(JointNameIndexing)joint_index].pos;
        buffers.read_door.unlock();
        return pos;
    }
    return std::nullopt;
}

std::optional<double> Monopod::get_velocity(const int &joint_index)
{
    if (is_initialized && Contains(read_joint_indexing, joint_index))
    {
        buffers.read_door.lock();
          double vel = buffers.read[(JointNameIndexing)joint_index].vel;
        buffers.read_door.unlock();
        return vel;
    }
    return std::nullopt;
}


std::optional<double> Monopod::get_acceleration(const int &joint_index)
{
    if (is_initialized && Contains(read_joint_indexing, joint_index))
    {
        buffers.read_door.lock();
          double acc = buffers.read[(JointNameIndexing)joint_index].acc;
        buffers.read_door.unlock();
        return acc;
    }
    return std::nullopt;
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

    if (is_initialized && Contains(read_joint_indexing, joint_index))
    {
        buffers.settings_door.lock();
          double max_target = buffers.settings[(JointNameIndexing)joint_index].max_torque_target;
        buffers.settings_door.unlock();
        return max_target;
    }
    return std::nullopt;
}

// ========================================
// Setters
// ========================================

bool Monopod::set_pid(const int &p, const int &i, const int &d, const int &joint_index)
{
    if (is_initialized && Contains(write_joint_indexing, joint_index))
    {
        buffers.pid_door.lock(); //Lock pid buffers
          buffers.pid[(JointNameIndexing)joint_index].p = p;
          buffers.pid[(JointNameIndexing)joint_index].i = i;
          buffers.pid[(JointNameIndexing)joint_index].d = d;
        buffers.pid_door.unlock(); //Unlock pid buffers
        return true;
    }
    return false;
}

bool Monopod::set_joint_position_limit(const double& max, const double& min, const int &joint_index)
{
    if (is_initialized && Contains(read_joint_indexing, joint_index))
    {
        buffers.settings_door.lock();
          buffers.settings[(JointNameIndexing)joint_index].position_limit.max = max;
          buffers.settings[(JointNameIndexing)joint_index].position_limit.min = min;
        buffers.settings_door.unlock();
        return true;
    }
    return false;
}

bool Monopod::set_joint_velocity_limit(const double& max, const double& min, const int &joint_index)
{
    if (is_initialized && Contains(read_joint_indexing, joint_index))
    {
        buffers.settings_door.lock();
          buffers.settings[(JointNameIndexing)joint_index].velocity_limit.max = max;
          buffers.settings[(JointNameIndexing)joint_index].velocity_limit.min = min;
        buffers.settings_door.unlock();
        return true;
    }
    return false;
}

bool Monopod::set_joint_acceleration_limit(const double& max, const double& min, const int &joint_index)
{
    if (is_initialized && Contains(read_joint_indexing, joint_index))
    {
        buffers.settings_door.lock();
          buffers.settings[(JointNameIndexing)joint_index].acceleration_limit.max = max;
          buffers.settings[(JointNameIndexing)joint_index].acceleration_limit.min = min;
        buffers.settings_door.unlock();
        return true;
    }
    return false;
}

bool Monopod::set_max_torque_target(const double &max_torque_target,  const int &joint_index)
{
    if (is_initialized && Contains(read_joint_indexing, joint_index))
    {
        buffers.settings_door.lock();
          // printf("new max torque: %f\n", max_torque_target );
          buffers.settings[(JointNameIndexing)joint_index].max_torque_target = max_torque_target;
        buffers.settings_door.unlock();
        return true;
    }
    return false;
}

bool Monopod::set_torque_target(const double &torque_target, const int joint_index)
{
    if (is_initialized && Contains(write_joint_indexing, joint_index))
    {
        buffers.settings_door.lock(); //Lock settings buffers
          double max_torque_target = buffers.settings[(JointNameIndexing)joint_index].max_torque_target;
        buffers.settings_door.unlock(); //Unlock settings buffers

        buffers.write_door.lock(); //Lock write buffers
          // Clip to max if over.
          if(std::abs(torque_target) > max_torque_target)
          {
              int force_dir = Monopod::sgn(torque_target);
              buffers.write[(JointNameIndexing)joint_index] = force_dir * max_torque_target;
          }
          else
          {
              buffers.write[(JointNameIndexing)joint_index] = torque_target;
          }
        buffers.write_door.unlock(); //Unlock write buffers
        return true;
    }
    return false;
}


bool Monopod::set_torque_targets(const std::vector<double> &torque_targets, const std::vector<int> &joint_indexes)
{
     // Note: if it fails the behaviour is undefined. For example if first 3 joints
     // are right but one bad index it will updatethe good ones the fail on the bad one
    const std::vector<int>& jointSerialization =
        joint_indexes.empty() ? write_joint_indexing : joint_indexes;

    if(torque_targets.size() != jointSerialization.size())
        // This means the inputs do not match
        return false;

    bool ok = true;
    buffers.write_door.lock(); //Lock write buffers
    buffers.settings_door.lock(); //Lock settings buffers

      for(size_t i = 0; i != torque_targets.size(); i++){
          if (is_initialized && Contains(write_joint_indexing, jointSerialization[i]))
          {
              double max_torque_target = buffers.settings[(JointNameIndexing)jointSerialization[i]].max_torque_target;
              // Clip to max if over.
              if(std::abs(torque_targets[i]) > max_torque_target)
              {
                  buffers.write[(JointNameIndexing)jointSerialization[i]] = sgn(torque_targets[i]) * max_torque_target;
              }
              else
              {
                  buffers.write[(JointNameIndexing)jointSerialization[i]] = torque_targets[i];
              }

              ok = ok && true;
          }
          else
          {
              ok = ok &&  false;
          }
      }

    buffers.settings_door.unlock(); //Unlock settings buffers
    buffers.write_door.unlock(); //Unlock write buffers
    return ok;
}

//===================================================================
// Private methods
//===================================================================

/**
* @brief this is a simple loop which runs at a kilohertz.
*
*/
void Monopod::loop()
{
  real_time_tools::Spinner spinner;
  spinner.set_period(0.001);  // 1kz loop

  // size_t count = 0;
  // size_t count_in = 0;

  while (!stop_loop)
  {
      /*
      * Collect data
      */
      auto data_leg = leg_->get_measurements();
      auto data_planarizer = planarizer_->get_measurements();
      /*
      * This section gets data from encoder joints then
      */
      std::vector<double> cur_pos;
      std::vector<double> cur_vel;

      cur_pos.reserve(read_joint_indexing.size());
      cur_vel.reserve(read_joint_indexing.size());
      // std::vector<double> cur_acc(read_joint_indexing.size(), 0);

      for (const auto &joint_index : read_joint_indexing)
      {
          if (Contains(write_joint_indexing, joint_index))
          {
            /* handle Leg data here */
            cur_pos.push_back(data_leg[(JointNameIndexing)joint_index][monopod_drivers::position]);
            cur_vel.push_back(data_leg[(JointNameIndexing)joint_index][monopod_drivers::velocity]);
          }
          else
          {
            /* handle Planarizer data here */
            cur_pos.push_back(data_planarizer[(JointNameIndexing)joint_index][monopod_drivers::position]);
            cur_vel.push_back(data_planarizer[(JointNameIndexing)joint_index][monopod_drivers::velocity]);
          }
      }

      /*
      * Check Limits of all the observations. If it is outside the limits we
      * have an issue and will want to enter a 'safe mode' instead of allowing
      * more actions. Safe mode will not prevent observations from being updated.
      */
      Monopod::JointLimit limit;
      bool valid = true;

      buffers.settings_door.lock(); //Lock settings buffers
      buffers.read_door.lock(); //Lock read buffers
      {
        for(size_t i = 0; i != read_joint_indexing.size(); i++){
            limit = buffers.settings[(JointNameIndexing)read_joint_indexing[i]].position_limit;
            valid = valid && in_range(cur_pos[i], limit.min, limit.max);
            buffers.read[(JointNameIndexing)read_joint_indexing[i]].pos = cur_pos[i];

            limit = buffers.settings[(JointNameIndexing)read_joint_indexing[i]].velocity_limit;
            valid = valid && in_range(cur_vel[i], limit.min, limit.max);
            buffers.read[(JointNameIndexing)read_joint_indexing[i]].vel = cur_vel[i];

            // limit = buffers.settings[(JointNameIndexing)read_joint_indexing[i]].acceleration_limit;
            // valid = valid && in_range(cur_acc[i], limit.min, limit.max);
            // buffers.read[(JointNameIndexing)read_joint_indexing[i]].acc = cur_acc[i];
        }
      }
      buffers.read_door.unlock(); //Unlock read buffers
      buffers.settings_door.unlock(); //Unlock settings buffers

      /*
      * Set the new torque values if observation was valid
      */

      if (valid) {
          auto torques = get_torque_targets({hip_joint, knee_joint});
          //TODO: This assumes torques has a value. This might not be the case those? In which case it must be handled

          leg_->set_target_torques(torques.value());
          leg_->send_target_torques();
      }
      else {
          // TODO: This sets torques to 0 as the safe mode right now? We might want to handle this in a mor creative way...
          std::vector<double> torques(0, 2);

          leg_->set_target_torques(torques);
          leg_->send_target_torques();
      }

      spinner.spin();
  }
}


} // end monopod_drivers namespace
