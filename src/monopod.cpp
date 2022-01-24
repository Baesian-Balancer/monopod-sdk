#include "monopod_sdk/monopod.hpp"
#include <stdexcept>

namespace monopod_drivers {

//===================================================================
// Public methods
//===================================================================

Monopod::Monopod() {
  stop_loop_limits = false;
  can_bus_ = std::make_shared<monopod_drivers::CanBus>("can0");
  can_bus_board_ =
      std::make_shared<monopod_drivers::CanBusControlBoards>(can_bus_);
}

Monopod::~Monopod() {
  stop_loop_limits = true;
  rt_thread_limits_.join();
}

bool Monopod::valid() { return !can_bus_board_->is_safemode(); }

void Monopod::reset() { can_bus_board_->reset_safemode(); }

bool Monopod::initialize(Mode monopod_mode) {

  encoder_joint_indexing = {};
  motor_joint_indexing = {};

  monopod_mode_ = monopod_mode;

  /* This switch state using run-into. This means Free has everything in fixed
   * connector. etc */
  switch (monopod_mode) {

  case Mode::Free:
    encoders_[boom_connector_joint] =
        create_encoder_module(boom_connector_joint);
    encoder_joint_indexing.push_back(boom_connector_joint);
    [[fallthrough]];

  case Mode::Fixed_connector:
    encoders_[planarizer_yaw_joint] =
        create_encoder_module(planarizer_yaw_joint);
    encoder_joint_indexing.push_back(planarizer_yaw_joint);
    [[fallthrough]];

  case Mode::Fixed:
    encoders_[planarizer_pitch_joint] =
        create_encoder_module(planarizer_pitch_joint);
    encoder_joint_indexing.push_back(planarizer_pitch_joint);
    [[fallthrough]];

  case Mode::motor_board: {

    auto motor_hip = create_motor_module(hip_joint);
    auto motor_knee = create_motor_module(knee_joint);

    encoders_[hip_joint] = motor_hip;
    encoders_[knee_joint] = motor_knee;

    motors_[hip_joint] = motor_hip;
    motors_[knee_joint] = motor_knee;

    leg_ = std::make_unique<monopod_drivers::Leg>(motor_hip, motor_knee);

    encoder_joint_indexing.push_back(hip_joint);
    encoder_joint_indexing.push_back(knee_joint);
    motor_joint_indexing.push_back(hip_joint);
    motor_joint_indexing.push_back(knee_joint);
    break;
  }

  case Mode::encoder_board1:
    encoders_[planarizer_pitch_joint] =
        create_encoder_module(planarizer_pitch_joint);
    encoders_[planarizer_yaw_joint] =
        create_encoder_module(planarizer_yaw_joint);

    encoder_joint_indexing.push_back(planarizer_pitch_joint);
    encoder_joint_indexing.push_back(planarizer_yaw_joint);
    break;

  case Mode::encoder_board2:
    encoders_[boom_connector_joint] =
        create_encoder_module(boom_connector_joint);

    encoder_joint_indexing.push_back(boom_connector_joint);
    break;
  }

  calibrate();
  is_initialized = true;
  return initialized();
}

void Monopod::start_loop() {
  if (is_initialized) {
    rt_thread_limits_.create_realtime_thread(&Monopod::loop, this);
  } else {
    throw std::runtime_error(
        "Need to initialize monopod_sdk before starting the realtime loop.");
  }
}

void Monopod::calibrate(const double &hip_home_offset_rad,
                        const double &knee_home_offset_rad) {

  // todo: Make calibration more robust??
  // only calibrate leg if it is active
  // need to calibrate or zero encoders
  if (!(monopod_mode_ == Mode::encoder_board1 ||
        monopod_mode_ == Mode::encoder_board2)) {
    leg_->calibrate(hip_home_offset_rad, knee_home_offset_rad);
  }
}

bool Monopod::initialized() { return is_initialized; }

bool Monopod::is_joint_controllable(const int joint_index) {
  return is_initialized && Contains(motor_joint_indexing, joint_index);
}

// ========================================
// Getters
// ========================================

std::string Monopod::get_model_name() const { return "monopod"; }

std::unordered_map<std::string, int> Monopod::get_joint_names() const {
  std::unordered_map<std::string, int> joint_names_;

  for (auto const &pair : joint_names) {
    if (Contains(encoder_joint_indexing, pair.second) ||
        Contains(motor_joint_indexing, pair.second)) {
      joint_names_[pair.first] = pair.second;
    }
  }
  return joint_names_;
}

std::optional<PID> Monopod::get_pid(const int &joint_index) {
  if (is_initialized && Contains(motor_joint_indexing, joint_index)) {
    // todo Implement PID read/write
  }
  return std::nullopt;
}

std::optional<JointLimit>
Monopod::get_joint_position_limit(const int &joint_index) {
  if (is_initialized && Contains(encoder_joint_indexing, joint_index)) {
    return encoders_[joint_index]->get_limit(position);
  }
  return std::nullopt;
}

std::optional<JointLimit>
Monopod::get_joint_velocity_limit(const int &joint_index) {
  if (is_initialized && Contains(encoder_joint_indexing, joint_index)) {
    return encoders_[joint_index]->get_limit(velocity);
  }
  return std::nullopt;
}

std::optional<JointLimit>
Monopod::get_joint_acceleration_limit(const int &joint_index) {
  if (is_initialized && Contains(encoder_joint_indexing, joint_index)) {
    return encoders_[joint_index]->get_limit(acceleration);
  }
  return std::nullopt;
}

std::optional<double> Monopod::get_torque_target(const int &joint_index) {
  if (is_initialized && Contains(motor_joint_indexing, joint_index)) {
    return motors_[joint_index]->get_measured_torque();
  }
  return std::nullopt;
}

std::optional<Vector<double>>
Monopod::get_torque_targets(const Vector<int> &joint_indexes) {
  const Vector<int> &jointSerialization =
      joint_indexes.empty() ? motor_joint_indexing : joint_indexes;

  Vector<double> data;
  data.reserve(jointSerialization.size());
  for (auto &joint_index : jointSerialization) {
    if (is_initialized && Contains(motor_joint_indexing, joint_index)) {
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
  if (is_initialized && Contains(encoder_joint_indexing, joint_index)) {
    return encoders_[joint_index]->get_measured_index_angle();
  }
  return std::nullopt;
}

std::optional<double> Monopod::get_velocity(const int &joint_index) {
  if (is_initialized && Contains(encoder_joint_indexing, joint_index)) {
    return encoders_[joint_index]->get_measured_velocity();
  }
  return std::nullopt;
}

std::optional<double> Monopod::get_acceleration(const int &joint_index) {
  if (is_initialized && Contains(encoder_joint_indexing, joint_index)) {
    return encoders_[joint_index]->get_measured_acceleration();
  }
  return std::nullopt;
}

std::optional<Vector<double>>
Monopod::get_positions(const Vector<int> &joint_indexes) {
  auto lambda = [this](int joint_index) -> double {
    return encoders_[joint_index]->get_measured_index_angle();
  };

  return getJointDataSerialized(this, joint_indexes, lambda);
}

std::optional<Vector<double>>
Monopod::get_velocities(const Vector<int> &joint_indexes) {
  auto lambda = [this](int joint_index) -> double {
    return encoders_[joint_index]->get_measured_velocity();
  };

  return getJointDataSerialized(this, joint_indexes, lambda);
}

std::optional<Vector<double>>
Monopod::get_accelerations(const Vector<int> &joint_indexes) {
  auto lambda = [this](int joint_index) -> double {
    return encoders_[joint_index]->get_measured_acceleration();
  };

  return getJointDataSerialized(this, joint_indexes, lambda);
}

std::optional<double> Monopod::get_max_torque_target(const int &joint_index) {

  if (is_initialized && Contains(motor_joint_indexing, joint_index)) {
    return motors_[joint_index]->get_max_torque();
  }
  // Todo: make sure there isnt any issue with returning null optional for max
  // torque on a read only joint...
  return std::nullopt;
}

// ========================================
// Setters
// ========================================

bool Monopod::set_pid(const int &p, const int &i, const int &d,
                      const int &joint_index) {
  if (is_initialized && Contains(motor_joint_indexing, joint_index)) {
    // todo Implement PID read/write
    PID pid(p, i, d);
    // motor_->set_pid(pid);
    return true;
  }
  return false;
}

bool Monopod::set_joint_position_limit(const double &max, const double &min,
                                       const int &joint_index) {
  if (is_initialized && Contains(encoder_joint_indexing, joint_index)) {
    JointLimit limit(min, max);
    encoders_[joint_index]->set_limit(position, limit);
    return true;
  }
  return false;
}

bool Monopod::set_joint_velocity_limit(const double &max, const double &min,
                                       const int &joint_index) {
  if (is_initialized && Contains(encoder_joint_indexing, joint_index)) {
    JointLimit limit(min, max);
    encoders_[joint_index]->set_limit(velocity, limit);
    return true;
  }
  return false;
}

bool Monopod::set_joint_acceleration_limit(const double &max, const double &min,
                                           const int &joint_index) {
  if (is_initialized && Contains(encoder_joint_indexing, joint_index)) {
    JointLimit limit(min, max);
    encoders_[joint_index]->set_limit(acceleration, limit);
    return true;
  }
  return false;
}

bool Monopod::set_max_torque_target(const double &max_torque_target,
                                    const int &joint_index) {
  if (is_initialized && Contains(motor_joint_indexing, joint_index)) {
    motors_[joint_index]->set_max_torque(max_torque_target);
    return true;
  } else if (is_initialized && Contains(encoder_joint_indexing, joint_index)) {
    return true;
  }
  return false;
}

bool Monopod::set_torque_target(const double &torque_target,
                                const int joint_index) {
  if (is_initialized && Contains(motor_joint_indexing, joint_index)) {
    /* automatically clip torque to max in joint module */
    motors_[joint_index]->set_torque(torque_target);
    motors_[joint_index]->send_torque();

    return true;
  }
  return false;
}

bool Monopod::set_torque_targets(const Vector<double> &torque_targets,
                                 const Vector<int> &joint_indexes) {

  const Vector<int> &jointSerialization =
      joint_indexes.empty() ? motor_joint_indexing : joint_indexes;

  if (torque_targets.size() != jointSerialization.size())
    return false;

  bool ok = true;

  for (size_t i = 0; i != torque_targets.size(); i++) {
    if (is_initialized &&
        Contains(motor_joint_indexing, jointSerialization[i])) {

      motors_[jointSerialization[i]]->set_torque(torque_targets[i]);
      motors_[jointSerialization[i]]->send_torque();

      ok = ok && true;
    } else {
      ok = ok && false;
    }
  }

  return ok;
}

/**
 * @brief This is a 100Hz loop that checks the limits of all joints. This is
 * done to make sure the monopod is not in a vulnerable state. Do not want to
 * break the robot.
 */
void Monopod::loop() {
  real_time_tools::Spinner spinner;
  spinner.set_period(0.01); // 100hz loop
  while (!stop_loop_limits) {
    bool in_limits = true;
    for (const auto &joint_index : encoder_joint_indexing) {
      in_limits = in_limits && encoders_[joint_index]->check_limits();
    }
    if (!in_limits) {
      can_bus_board_->enter_safemode();
    }
    // spin the RT loop.
    spinner.spin();
  }
}

} // namespace monopod_drivers
