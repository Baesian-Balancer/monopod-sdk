#include "monopod_sdk/monopod.hpp"
#include <stdexcept>

namespace monopod_drivers {

//===================================================================
// Public methods
//===================================================================

Monopod::Monopod() {
  can_bus_ = std::make_shared<monopod_drivers::CanBus>("can0");
  can_bus_board_ =
      std::make_shared<monopod_drivers::CanBusControlBoards>(can_bus_);
}

Monopod::~Monopod() {}

bool Monopod::initialize(Mode monopod_mode) {

  read_joint_indexing = {};
  write_joint_indexing = {};

  monopod_mode_ = monopod_mode;

  /* This switch state using run-into. This means Free has everything in fixed
   * connector. etc */
  switch (monopod_mode) {

  case Mode::Free: {
    encoders_[boom_connector_joint] =
        create_encoder_module(boom_connector_joint);
    read_joint_indexing.push_back(boom_connector_joint);
  }

  case Mode::Fixed_connector: {

    encoders_[planarizer_yaw_joint] =
        create_encoder_module(planarizer_yaw_joint);
    read_joint_indexing.push_back(planarizer_yaw_joint);
  }

  case Mode::Fixed: {
    encoders_[planarizer_pitch_joint] =
        create_encoder_module(planarizer_pitch_joint);
    read_joint_indexing.push_back(planarizer_pitch_joint);
  }

  case Mode::motor_board: {

    auto motor_hip = create_motor_module(hip_joint);
    auto motor_knee = create_motor_module(knee_joint);

    encoders_[hip_joint] = motor_hip;
    encoders_[knee_joint] = motor_knee;

    motors_[hip_joint] = motor_hip;
    motors_[knee_joint] = motor_knee;

    leg_ = std::make_unique<monopod_drivers::Leg>(motor_hip, motor_knee);

    read_joint_indexing.push_back(hip_joint);
    read_joint_indexing.push_back(knee_joint);
    write_joint_indexing.push_back(hip_joint);
    write_joint_indexing.push_back(knee_joint);
    break;
  }

  case Mode::encoder_board1: {
    encoders_[planarizer_pitch_joint] =
        create_encoder_module(planarizer_pitch_joint);
    encoders_[planarizer_yaw_joint] =
        create_encoder_module(planarizer_yaw_joint);

    read_joint_indexing.push_back(planarizer_pitch_joint);
    read_joint_indexing.push_back(planarizer_yaw_joint);
    break;
  }

  case Mode::encoder_board2: {
    encoders_[boom_connector_joint] =
        create_encoder_module(boom_connector_joint);

    read_joint_indexing.push_back(boom_connector_joint);
    break;
  }
  }

  calibrate();
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
    // todo Implement PID read/write
  }
  return std::nullopt;
}

std::optional<Monopod::JointLimit>
Monopod::get_joint_position_limit(const int &joint_index) {
  if (is_initialized && Contains(read_joint_indexing, joint_index)) {
    const Monopod::JointLimit limit;
    // todo get limit from the joint encoder here. something like
    // encoders_[joint_index]->get_limit(MeasurementIndex::position)
    return limit;
  }
  return std::nullopt;
}

std::optional<Monopod::JointLimit>
Monopod::get_joint_velocity_limit(const int &joint_index) {
  if (is_initialized && Contains(read_joint_indexing, joint_index)) {
    const Monopod::JointLimit limit;
    // todo get limit from the joint encoder here. something like
    // encoders_[joint_index]->get_limit(MeasurementIndex::velocity)
    return limit;
  }
  return std::nullopt;
}

std::optional<Monopod::JointLimit>
Monopod::get_joint_acceleration_limit(const int &joint_index) {
  if (is_initialized && Contains(read_joint_indexing, joint_index)) {
    const Monopod::JointLimit limit;
    // todo get limit from the joint encoder here. something like
    // encoders_[joint_index]->get_limit(MeasurementIndex::acceleration)
    return limit;
  }
  return std::nullopt;
}

std::optional<double> Monopod::get_torque_target(const int &joint_index) {
  if (is_initialized && Contains(write_joint_indexing, joint_index)) {
    double torque_target = motors_[joint_index]->get_measured_torque();
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
  for (auto &joint_index : jointSerialization) {
    if (is_initialized && Contains(write_joint_indexing, joint_index)) {
      // note: maybe we cna call the single version here.
      data.push_back(motors_[joint_index]->get_measured_torque());
      continue;
    } else {
      return std::nullopt;
    }
  }
  return data;
}

std::optional<double> Monopod::get_position(const int &joint_index) {
  if (is_initialized && Contains(read_joint_indexing, joint_index)) {
    return encoders_[joint_index]->get_measured_index_angle();
  }
  return std::nullopt;
}

std::optional<double> Monopod::get_velocity(const int &joint_index) {
  if (is_initialized && Contains(read_joint_indexing, joint_index)) {
    return encoders_[joint_index]->get_measured_velocity();
  }
  return std::nullopt;
}

std::optional<double> Monopod::get_acceleration(const int &joint_index) {
  if (is_initialized && Contains(read_joint_indexing, joint_index)) {
    return encoders_[joint_index]->get_measured_acceleration();
  }
  return std::nullopt;
}

std::optional<std::vector<double>>
Monopod::get_positions(const std::vector<int> &joint_indexes) {
  auto lambda = [this](int joint_index) -> double {
    return encoders_[joint_index]->get_measured_index_angle();
  };

  return getJointDataSerialized(this, joint_indexes, lambda);
}

std::optional<std::vector<double>>
Monopod::get_velocities(const std::vector<int> &joint_indexes) {
  auto lambda = [this](int joint_index) -> double {
    return encoders_[joint_index]->get_measured_velocity();
  };

  return getJointDataSerialized(this, joint_indexes, lambda);
}

std::optional<std::vector<double>>
Monopod::get_accelerations(const std::vector<int> &joint_indexes) {
  auto lambda = [this](int joint_index) -> double {
    return encoders_[joint_index]->get_measured_acceleration();
  };

  return getJointDataSerialized(this, joint_indexes, lambda);
}

std::optional<double> Monopod::get_max_torque_target(const int &joint_index) {

  if (is_initialized && Contains(read_joint_indexing, joint_index)) {
    // todo figure out where/how to handle max torque limiting/clipping.
    double max_target = 0;
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
    // todo Implement PID read/write
    return true;
  }
  return false;
}

bool Monopod::set_joint_position_limit(const double &max, const double &min,
                                       const int &joint_index) {
  if (is_initialized && Contains(read_joint_indexing, joint_index)) {
    // todo Need to figure out where the liits sit.
    return true;
  }
  return false;
}

bool Monopod::set_joint_velocity_limit(const double &max, const double &min,
                                       const int &joint_index) {
  if (is_initialized && Contains(read_joint_indexing, joint_index)) {
    // todo Need to figure out where the liits sit.
    return true;
  }
  return false;
}

bool Monopod::set_joint_acceleration_limit(const double &max, const double &min,
                                           const int &joint_index) {
  if (is_initialized && Contains(read_joint_indexing, joint_index)) {
    // todo Need to figure out where the liits sit.
    return true;
  }
  return false;
}

bool Monopod::set_max_torque_target(const double &max_torque_target,
                                    const int &joint_index) {
  if (is_initialized && Contains(read_joint_indexing, joint_index)) {
    // todo Need to figure out where the max torque and truncation sit.
    return true;
  }
  return false;
}

bool Monopod::set_torque_target(const double &torque_target,
                                const int joint_index) {
  if (is_initialized && Contains(write_joint_indexing, joint_index)) {

    double max_torque_target = 0;
    // todo : handle the max torque here.
    double limited_torque_target = 0;
    // Clip to max if over.
    if (std::abs(torque_target) > max_torque_target) {

      int force_dir = Monopod::sgn(torque_target);
      limited_torque_target = force_dir * max_torque_target;

    } else {

      limited_torque_target = torque_target;
    }

    motors_[joint_index]->set_torque(limited_torque_target);
    motors_[joint_index]->send_torque();

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

  for (size_t i = 0; i != torque_targets.size(); i++) {

    double limited_torque_target = 0;

    if (is_initialized &&
        Contains(write_joint_indexing, jointSerialization[i])) {
      // todo : handle the max torque here.
      double max_torque_target = 0;

      // Clip to max if over.
      if (std::abs(torque_targets[i]) > max_torque_target) {
        limited_torque_target = sgn(torque_targets[i]) * max_torque_target;
      } else {
        limited_torque_target = torque_targets[i];
      }

      motors_[jointSerialization[i]]->set_torque(limited_torque_target);
      motors_[jointSerialization[i]]->send_torque();

      ok = ok && true;
    } else {
      ok = ok && false;
    }
  }

  return ok;
}

} // namespace monopod_drivers
