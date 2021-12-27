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
    // // First of all one need to initialize the communication with the can bus.
    // canbus_.leg = std::make_shared<blmc_drivers::CanBus>("can0");
    // canbus_.planarizer1 = std::make_shared<blmc_drivers::CanBus>("can1");
    // canbus_.planarizer2 = std::make_shared<blmc_drivers::CanBus>("can2");
    // rt_printf("Canbus is set up \n");
    //
    // // Then we create a motor board object that will use the can bus in order
    // // communicate between this application and the actual motor board.
    // // Important: the blmc motors are alinged during this stage.
    // auto motor_board = std::make_shared<blmc_drivers::CanBusMotorBoard>(canbus_.leg);
    // auto planarizer_board1 = std::make_shared<blmc_drivers::CanBusMotorBoard>(canbus_.planarizer1);
    // auto planarizer_board2 = std::make_shared<blmc_drivers::CanBusMotorBoard>(canbus_.planarizer2);
    // rt_printf("Board are set up \n");
    //
    // // create the motor object that have an index that define the port on which
    // // they are plugged on the motor board. This object takes also a MotorBoard
    // // object to be able to get the sensors and send the control consistantly.
    // // These safe motors have the ability to bound the current that is given
    // // as input.
    // auto motor_hip = std::make_shared<blmc_drivers::SafeMotor>(motor_board, 0);
    // auto motor_knee = std::make_shared<blmc_drivers::SafeMotor>(motor_board, 1);
    // rt_printf("motors are set up \n");
    //
    // auto encoder_planarizer_yaw = std::make_shared<monopod_drivers::Encoder>(planarizer_board1, 0);
    // auto encoder_planarizer_pitch = std::make_shared<monopod_drivers::Encoder>(planarizer_board1, 1);
    // auto encoder_connector = std::make_shared<monopod_drivers::Encoder>(planarizer_board2, 0);
    // rt_printf("encoders are set up. \n");
    //
    // leg_ = std::make_unique<monopod_drivers::Leg>(motor_hip, motor_knee);
    // planarizer_ = std::make_unique<monopod_drivers::Planarizer>(encoder_planarizer_yaw, encoder_planarizer_pitch, encoder_connector);
    // rt_printf("initialization is complete. \n");
    //
    // // TODO: Auto detect the number of boards for the encoders. want to be able to detect if only fixed hip with single TI for encoders
    // // Or if only 1 encoder is attached to the first planarizer board (fixed yaw). This will be a little bit complicated as it will change the
    // // Logic used below too

    is_initialized = true;
    return initialized();
}

bool Monopod::initialized()
{
    return is_initialized;
}

void Monopod::start_loop()
{
    rt_thread_.create_realtime_thread(&Monopod::loop, this);
}

bool Monopod::is_joint_controllable(const int joint_index)
{
    if (is_initialized && Contains(motor_joint_indexing, joint_index))
    {
          return true;
    }
    return false;
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
    if (is_initialized && Contains(motor_joint_indexing, joint_index))
    {
          buffers.pid_door.lock(); //Lock pid buffers
            const PID _pid = buffers.pid[(JointNameIndexing)joint_index];
          buffers.pid_door.unlock(); //unLock pid buffers
          return _pid;
    }
    return std::nullopt;

}

std::optional<Monopod::JointLimit> Monopod::get_joint_position_limit(const int &joint_index)
{
    if (is_initialized && Contains(encoder_joint_indexing, joint_index))
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
    if (is_initialized && Contains(encoder_joint_indexing, joint_index))
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
    if (is_initialized && Contains(encoder_joint_indexing, joint_index))
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
    if (is_initialized && Contains(motor_joint_indexing, joint_index))
    {
          buffers.write_door.lock(); //Lock write buffers
            double torque_target = buffers.write[(JointNameIndexing)joint_index];
          buffers.write_door.unlock(); //unLock write buffers
          return torque_target;
    }
    return std::nullopt;
}

std::optional<std::vector<double>> Monopod::get_torque_targets(const std::vector<int> &joint_indexes)
{
    const std::vector<int>& jointSerialization =
        joint_indexes.empty() ? motor_joint_indexing : joint_indexes;

    std::vector<double> data;
    data.reserve(jointSerialization.size());
    buffers.write_door.lock(); //Lock write buffers
      for(auto& joint_index : jointSerialization){
          if (is_initialized && Contains(motor_joint_indexing, joint_index))
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
    buffers.write_door.unlock(); //unLock write buffers
    return data;

}

std::optional<double> Monopod::get_position(const int &joint_index)
{
    if (is_initialized && Contains(encoder_joint_indexing, joint_index))
    {
        buffers.read_door.lock();
          double position = buffers.read[(JointNameIndexing)joint_index].pos;
        buffers.read_door.unlock();
        return position;
    }
    return std::nullopt;
}

std::optional<double> Monopod::get_velocity(const int &joint_index)
{
    if (is_initialized && Contains(encoder_joint_indexing, joint_index))
    {
        buffers.read_door.lock();
          double velocity = buffers.read[(JointNameIndexing)joint_index].vel;
        buffers.read_door.unlock();
        return velocity;
    }
    return std::nullopt;
}


std::optional<double> Monopod::get_acceleration(const int &joint_index)
{
    if (is_initialized && Contains(encoder_joint_indexing, joint_index))
    {
        buffers.read_door.lock();
          double acceleration = buffers.read[(JointNameIndexing)joint_index].acc;
        buffers.read_door.unlock();
        return acceleration;
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

    if (is_initialized && Contains(encoder_joint_indexing, joint_index))
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
    if (is_initialized && Contains(motor_joint_indexing, joint_index))
    {
        buffers.pid_door.lock(); //Lock pid buffers
          buffers.pid[(JointNameIndexing)joint_index].p = p;
          buffers.pid[(JointNameIndexing)joint_index].i = i;
          buffers.pid[(JointNameIndexing)joint_index].d = d;
        buffers.pid_door.unlock(); //unLock pid buffers
        return true;
    }
    return false;
}

bool Monopod::set_joint_position_limit(const double& max, const double& min, const int &joint_index)
{
    if (is_initialized && Contains(encoder_joint_indexing, joint_index))
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
    if (is_initialized && Contains(encoder_joint_indexing, joint_index))
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
    if (is_initialized && Contains(encoder_joint_indexing, joint_index))
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
    if (is_initialized && Contains(encoder_joint_indexing, joint_index))
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
    if (is_initialized && Contains(motor_joint_indexing, joint_index))
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
    return false;
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
    buffers.settings_door.lock(); //Lock settings buffers

      for(std::vector<int>::size_type i = 0; i != torque_targets.size(); i++){
          if (is_initialized && Contains(motor_joint_indexing, jointSerialization[i]))
          {
              double max_torque_target = buffers.settings[(JointNameIndexing)jointSerialization[i]].max_torque_target;
              // Clip to max if over.
              if(std::abs(torque_targets[i]) > max_torque_target)
              {
                  int force_dir = Monopod::sgn(torque_targets[i]);
                  buffers.write[(JointNameIndexing)jointSerialization[i]] = force_dir * max_torque_target;
              }else{
                  buffers.write[(JointNameIndexing)jointSerialization[i]] = torque_targets[i];
              }

              ok = ok && true;
          }
          else{
            ok = ok &&  false;
          }
      }

    buffers.settings_door.unlock(); //unLock settings buffers
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

      // if (is_initialized) {
      //
      //   // Get data from board
      //
      //   // Check Limits
      //
      //   // Set Torque
      //
      // }else{
      //   rt_printf("Monopod_sdk is not initialized. Waiting for initialization.");
      //   real_time_tools::Timer::sleep_sec(1.0);
      // }


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
