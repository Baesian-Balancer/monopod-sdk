#include "monopod_sdk/monopod.hpp"
#include <stdexcept>

namespace monopod_drivers {

//===================================================================
// Public methods
//===================================================================

Monopod::Monopod() {
  stop_loop = false;
  read_joint_indexing = {hip_joint, knee_joint};
  write_joint_indexing = {hip_joint, knee_joint};
  can_bus_ = std::make_shared<monopod_drivers::CanBus>("can0");
  can_bus_board_ =
      std::make_shared<monopod_drivers::CanBusControlBoards>(can_bus_);

  leg_ = std::make_unique<monopod_drivers::Leg>(can_bus_board_);
  planarizer_ = std::make_unique<monopod_drivers::Planarizer>(can_bus_board_);
}

Monopod::~Monopod() { stop_loop = true; }

bool Monopod::initialize(int num_joints, const double &hip_home_offset_rad,
                         const double &knee_home_offset_rad) {
  switch (num_joints) {
  // Todo: make this more simplified...
  case 5:
    read_joint_indexing = {hip_joint, knee_joint, boom_connector_joint,
                           planarizer_yaw_joint, planarizer_pitch_joint};
    leg_->initialize();
    /* Remove the leg joints*/
    planarizer_->initialize(num_joints - NUMBER_LEG_JOINTS);
    rt_printf("Controlling the leg and %d joints.\n",
              num_joints - NUMBER_LEG_JOINTS);
    break;
  case 4:
    read_joint_indexing = {hip_joint, knee_joint, planarizer_yaw_joint,
                           planarizer_pitch_joint};
    leg_->initialize();
    /* Remove the leg joints*/
    planarizer_->initialize(num_joints - NUMBER_LEG_JOINTS);
    rt_printf("Controlling the leg and %d joints.\n",
              num_joints - NUMBER_LEG_JOINTS);
    break;
  case 3:
    read_joint_indexing = {hip_joint, knee_joint, planarizer_pitch_joint};
    leg_->initialize();
    /* Remove the leg joints*/
    planarizer_->initialize(num_joints - NUMBER_LEG_JOINTS);
    rt_printf("Controlling the leg and %d joints.\n",
              num_joints - NUMBER_LEG_JOINTS);
    break;
  case 2:
    leg_->initialize();
    rt_printf("Controlling only the leg\n");
    break;
  default:
    throw std::runtime_error(
        "only Supports 2 (only leg), 3 (fixed hip_joint and "
        "planarizer_yaw_joint),4 (fixed hip_joint), 5 (free) joints.\n");
  }

  calibrate(hip_home_offset_rad, knee_home_offset_rad);
  num_joints_ = num_joints;
  is_initialized = true;
  return initialized();
}

void Monopod::calibrate(const double &hip_home_offset_rad,
                        const double &knee_home_offset_rad) {

  // todo: Make calibration more robust??
  leg_->calibrate(hip_home_offset_rad, knee_home_offset_rad);
}

bool Monopod::initialized() { return is_initialized; }

bool Monopod::is_joint_controllable(const int joint_index) {
  return is_initialized && Contains(write_joint_indexing, joint_index);
}

// ========================================
// Getters
// ========================================

std::string Monopod::get_model_name() const { return "monopod"; }

std::unordered_map<std::string, int> Monopod::get_joint_names() const {
  std::unordered_map<std::string, int> joint_names_;

  for (auto const &pair : joint_names) {
    if (Contains(read_joint_indexing, pair.second) ||
        Contains(write_joint_indexing, pair.second)) {
      joint_names_[pair.first] = pair.second;
    }
  }
  return joint_names_;
}

std::optional<Monopod::PID> Monopod::get_pid(const int &joint_index) {
  if (is_initialized && Contains(write_joint_indexing, joint_index)) {
    buffers.pid_door.lock(); // Lock pid buffers
    const PID _pid = buffers.pid[(JointNameIndexing)joint_index];
    buffers.pid_door.unlock(); // Unlock pid buffers
    return _pid;
  }
  return std::nullopt;
}

std::optional<Monopod::JointLimit>
Monopod::get_joint_position_limit(const int &joint_index) {
  if (is_initialized && Contains(read_joint_indexing, joint_index)) {
    buffers.settings_door.lock();
    const Monopod::JointLimit limit =
        buffers.settings[(JointNameIndexing)joint_index].position_limit;
    buffers.settings_door.unlock();
    return limit;
  }
  return std::nullopt;
}

std::optional<Monopod::JointLimit>
Monopod::get_joint_velocity_limit(const int &joint_index) {
  if (is_initialized && Contains(read_joint_indexing, joint_index)) {
    buffers.settings_door.lock();
    const Monopod::JointLimit limit =
        buffers.settings[(JointNameIndexing)joint_index].velocity_limit;
    buffers.settings_door.unlock();
    return limit;
  }
  return std::nullopt;
}

std::optional<Monopod::JointLimit>
Monopod::get_joint_acceleration_limit(const int &joint_index) {
  if (is_initialized && Contains(read_joint_indexing, joint_index)) {
    buffers.settings_door.lock();
    const Monopod::JointLimit limit =
        buffers.settings[(JointNameIndexing)joint_index].acceleration_limit;
    buffers.settings_door.unlock();
    return limit;
  }
  return std::nullopt;
}

std::optional<double> Monopod::get_torque_target(const int &joint_index) {
  if (is_initialized && Contains(write_joint_indexing, joint_index)) {
    buffers.write_door.lock(); // Lock write buffers
    double torque_target = buffers.write[(JointNameIndexing)joint_index];
    buffers.write_door.unlock(); // Unlock write buffers
    return torque_target;
  }
  return std::nullopt;
}

std::optional<std::vector<double>>
Monopod::get_torque_targets(const std::vector<int> &joint_indexes) {
  const std::vector<int> &jointSerialization =
      joint_indexes.empty() ? write_joint_indexing : joint_indexes;

  std::vector<double> data;
  data.reserve(jointSerialization.size());
  buffers.write_door.lock(); // Lock write buffers
  for (auto &joint_index : jointSerialization) {
    if (is_initialized && Contains(write_joint_indexing, joint_index)) {
      data.push_back(buffers.write[(JointNameIndexing)joint_index]);
      continue;
    } else {
      buffers.write_door.unlock();
      return std::nullopt;
    }
  }
  buffers.write_door.unlock(); // Unlock write buffers
  return data;
}

std::optional<double> Monopod::get_position(const int &joint_index) {
  if (is_initialized && Contains(read_joint_indexing, joint_index)) {
    buffers.read_door.lock();
    double pos = buffers.read[(JointNameIndexing)joint_index].pos;
    buffers.read_door.unlock();
    return pos;
  }
  return std::nullopt;
}

std::optional<double> Monopod::get_velocity(const int &joint_index) {
  if (is_initialized && Contains(read_joint_indexing, joint_index)) {
    buffers.read_door.lock();
    double vel = buffers.read[(JointNameIndexing)joint_index].vel;
    buffers.read_door.unlock();
    return vel;
  }
  return std::nullopt;
}

std::optional<double> Monopod::get_acceleration(const int &joint_index) {
  if (is_initialized && Contains(read_joint_indexing, joint_index)) {
    buffers.read_door.lock();
    double acc = buffers.read[(JointNameIndexing)joint_index].acc;
    buffers.read_door.unlock();
    return acc;
  }
  return std::nullopt;
}

std::optional<std::vector<double>>
Monopod::get_positions(const std::vector<int> &joint_indexes) {
  auto lambda = [this](int joint_index) -> double {
    return buffers.read[(JointNameIndexing)joint_index].pos;
  };

  buffers.read_door.lock();
  auto data = getJointDataSerialized(this, joint_indexes, lambda);
  buffers.read_door.unlock();
  return data;
}

std::optional<std::vector<double>>
Monopod::get_velocities(const std::vector<int> &joint_indexes) {
  auto lambda = [this](int joint_index) -> double {
    return buffers.read[(JointNameIndexing)joint_index].vel;
  };

  buffers.read_door.lock();
  auto data = getJointDataSerialized(this, joint_indexes, lambda);
  buffers.read_door.unlock();
  return data;
}

std::optional<std::vector<double>>
Monopod::get_accelerations(const std::vector<int> &joint_indexes) {
  auto lambda = [this](int joint_index) -> double {
    return buffers.read[(JointNameIndexing)joint_index].acc;
  };

  buffers.read_door.lock();
  auto data = getJointDataSerialized(this, joint_indexes, lambda);
  buffers.read_door.unlock();
  return data;
}

std::optional<double> Monopod::get_max_torque_target(const int &joint_index) {

  if (is_initialized && Contains(read_joint_indexing, joint_index)) {
    buffers.settings_door.lock();
    double max_target =
        buffers.settings[(JointNameIndexing)joint_index].max_torque_target;
    buffers.settings_door.unlock();
    return max_target;
  }
  return std::nullopt;
}

// ========================================
// Setters
// ========================================

bool Monopod::set_pid(const int &p, const int &i, const int &d,
                      const int &joint_index) {
  if (is_initialized && Contains(write_joint_indexing, joint_index)) {
    buffers.pid_door.lock(); // Lock pid buffers
    buffers.pid[(JointNameIndexing)joint_index].p = p;
    buffers.pid[(JointNameIndexing)joint_index].i = i;
    buffers.pid[(JointNameIndexing)joint_index].d = d;
    buffers.pid_door.unlock(); // Unlock pid buffers
    return true;
  }
  return false;
}

bool Monopod::set_joint_position_limit(const double &max, const double &min,
                                       const int &joint_index) {
  if (is_initialized && Contains(read_joint_indexing, joint_index)) {
    buffers.settings_door.lock();
    buffers.settings[(JointNameIndexing)joint_index].position_limit.max = max;
    buffers.settings[(JointNameIndexing)joint_index].position_limit.min = min;
    buffers.settings_door.unlock();
    return true;
  }
  return false;
}

bool Monopod::set_joint_velocity_limit(const double &max, const double &min,
                                       const int &joint_index) {
  if (is_initialized && Contains(read_joint_indexing, joint_index)) {
    buffers.settings_door.lock();
    buffers.settings[(JointNameIndexing)joint_index].velocity_limit.max = max;
    buffers.settings[(JointNameIndexing)joint_index].velocity_limit.min = min;
    buffers.settings_door.unlock();
    return true;
  }
  return false;
}

bool Monopod::set_joint_acceleration_limit(const double &max, const double &min,
                                           const int &joint_index) {
  if (is_initialized && Contains(read_joint_indexing, joint_index)) {
    buffers.settings_door.lock();
    buffers.settings[(JointNameIndexing)joint_index].acceleration_limit.max =
        max;
    buffers.settings[(JointNameIndexing)joint_index].acceleration_limit.min =
        min;
    buffers.settings_door.unlock();
    return true;
  }
  return false;
}

bool Monopod::set_max_torque_target(const double &max_torque_target,
                                    const int &joint_index) {
  if (is_initialized && Contains(read_joint_indexing, joint_index)) {
    buffers.settings_door.lock();
    // printf("new max torque: %f\n", max_torque_target );
    buffers.settings[(JointNameIndexing)joint_index].max_torque_target =
        max_torque_target;
    buffers.settings_door.unlock();
    return true;
  }
  return false;
}

bool Monopod::set_torque_target(const double &torque_target,
                                const int joint_index) {
  if (is_initialized && Contains(write_joint_indexing, joint_index)) {
    buffers.settings_door.lock(); // Lock settings buffers
    double max_torque_target =
        buffers.settings[(JointNameIndexing)joint_index].max_torque_target;
    buffers.settings_door.unlock(); // Unlock settings buffers

    buffers.write_door.lock(); // Lock write buffers
    // Clip to max if over.
    if (std::abs(torque_target) > max_torque_target) {
      int force_dir = Monopod::sgn(torque_target);
      buffers.write[(JointNameIndexing)joint_index] =
          force_dir * max_torque_target;
    } else {
      buffers.write[(JointNameIndexing)joint_index] = torque_target;
    }
    buffers.write_door.unlock(); // Unlock write buffers
    return true;
  }
  return false;
}

bool Monopod::set_torque_targets(const std::vector<double> &torque_targets,
                                 const std::vector<int> &joint_indexes) {
  // Note: if it fails the behaviour is undefined. For example if first 3 joints
  // are right but one bad index it will updatethe good ones the fail on the bad
  // one
  const std::vector<int> &jointSerialization =
      joint_indexes.empty() ? write_joint_indexing : joint_indexes;

  if (torque_targets.size() != jointSerialization.size())
    // This means the inputs do not match
    return false;

  bool ok = true;
  buffers.write_door.lock();    // Lock write buffers
  buffers.settings_door.lock(); // Lock settings buffers

  for (size_t i = 0; i != torque_targets.size(); i++) {
    if (is_initialized &&
        Contains(write_joint_indexing, jointSerialization[i])) {
      double max_torque_target =
          buffers.settings[(JointNameIndexing)jointSerialization[i]]
              .max_torque_target;
      // Clip to max if over.
      if (std::abs(torque_targets[i]) > max_torque_target) {
        buffers.write[(JointNameIndexing)jointSerialization[i]] =
            sgn(torque_targets[i]) * max_torque_target;
      } else {
        buffers.write[(JointNameIndexing)jointSerialization[i]] =
            torque_targets[i];
      }

      ok = ok && true;
    } else {
      ok = ok && false;
    }
  }

  buffers.settings_door.unlock(); // Unlock settings buffers
  buffers.write_door.unlock();    // Unlock write buffers
  return ok;
}

} // namespace monopod_drivers
