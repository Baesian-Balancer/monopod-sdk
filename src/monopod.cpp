#include "monopod_sdk/monopod.hpp"
#include <cassert>
#include <stdexcept>

// Use (void) to silent unused warnings.
#define assertm(exp, msg) assert(((void)msg, exp))

namespace monopod_drivers {

//===================================================================
// Public methods
//===================================================================

Monopod::Monopod() {
  safety_loop_running = false;
  pause_safety_loop = false;
  current_state_ = MonopodState::NOT_INITIALIZED;
}

Monopod::~Monopod() {
  safety_loop_running = false;
  pause_safety_loop = false;
  rt_thread_safety_.join();
  reset(false);
}

bool Monopod::initialize(Mode monopod_mode, bool dummy_mode) {
  dummy_mode_ = dummy_mode;
  if (!dummy_mode) {
    can_bus_ = std::make_shared<monopod_drivers::CanBus>("can0");
    board_ = std::make_shared<monopod_drivers::CanBusControlBoards>(can_bus_);
    board_->reset();

  } else {
    board_ = std::make_shared<monopod_drivers::DummyControlBoards>();
  }

  encoder_joint_indexing = {};
  motor_joint_indexing = {};

  monopod_mode_ = monopod_mode;

  /* This switch state using run-into. This means Free has everything in fixed
   * connector. etc */
  switch (monopod_mode) {

  case Mode::FREE:
    encoders_[boom_connector_joint] =
        create_encoder_module(boom_connector_joint);
    encoder_joint_indexing.push_back(boom_connector_joint);
    [[fallthrough]];

  case Mode::FIXED_CONNECTOR:
    encoders_[planarizer_yaw_joint] =
        create_encoder_module(planarizer_yaw_joint);
    encoder_joint_indexing.push_back(planarizer_yaw_joint);
    [[fallthrough]];

  case Mode::FIXED:
    encoders_[planarizer_pitch_joint] =
        create_encoder_module(planarizer_pitch_joint);
    encoder_joint_indexing.push_back(planarizer_pitch_joint);
    [[fallthrough]];

  case Mode::MOTOR_BOARD: {

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

  case Mode::ENCODER_BOARD1:
    encoders_[planarizer_pitch_joint] =
        create_encoder_module(planarizer_pitch_joint);
    encoders_[planarizer_yaw_joint] =
        create_encoder_module(planarizer_yaw_joint);

    encoder_joint_indexing.push_back(planarizer_pitch_joint);
    encoder_joint_indexing.push_back(planarizer_yaw_joint);
    break;

  case Mode::ENCODER_BOARD2:
    encoders_[boom_connector_joint] =
        create_encoder_module(boom_connector_joint);

    encoder_joint_indexing.push_back(boom_connector_joint);
    break;
  }
  board_->wait_until_ready();

  current_state_ = motor_joint_indexing.empty() ? MonopodState::READ_ONLY
                                                : MonopodState::RUNNING;
  start_safety_loop();

  return initialized();
}

bool Monopod::initialized() const {
  return current_state_ != MonopodState::NOT_INITIALIZED;
}

// Monopod::ERROR_CODE Monopod::error_code(int method_type){}

bool Monopod::valid() {
  // invalid if it is in safemode or holding
  assertm(initialized(), "Requires monopod_sdk is initialized.");
  return !(board_->is_safemode() || current_state_ == MonopodState::HOLDING);
}

void Monopod::print(const Vector<int> &joint_indexes) const {
  assertm(initialized(), "Requires monopod_sdk is initialized.");
  const Vector<int> &jointSerialization =
      joint_indexes.empty() ? encoder_joint_indexing : joint_indexes;

  for (auto &joint_index : jointSerialization) {
    if (Contains(encoder_joint_indexing, joint_index)) {
      encoders_.at(joint_index)->print();
    }
  }
}

void Monopod::reset(const bool &move_to_zero) {
  // Make sure we are in a reset state before going to zero.
  assertm(initialized(), "Requires monopod_sdk is initialized.");
  if (current_state_ == MonopodState::HOLDING) {
    stop_hold_position();
  }
  board_->reset();
  if (move_to_zero) {
    // by default moves to home
    goto_position();
  }
  current_state_ = MonopodState::RUNNING;
}

bool Monopod::goto_position(const double &hip_home_position,
                            const double &knee_home_position) {
  assertm(initialized(), "Requires monopod_sdk is initialized.");
  if (current_state_ != MonopodState::RUNNING) {
    std::cerr << "Monopod::goto_position(): Tried to goto_position when "
                 "not in [MonopodState::RUNNING]. This means the "
                 "robot is HOLDING or in READ_ONLY mode with no active motors."
              << std::endl;
    return false;
  }

  // Disable limits to avoid triggering the safemode.
  pause_safety_loop = true;
  // Make sure we are in a reset state before going to zero.
  board_->reset();
  bool ok = leg_->goto_position(hip_home_position, knee_home_position);
  // Reset here pauses the motors again.
  board_->reset();
  // start limit loop again if it was active before
  pause_safety_loop = false;

  if (!ok) {
    std::cerr
        << "Monopod::goto_position(): Failed to goto_position when "
        << "exectuing the control. This most likely occoured if the observed "
        << "error between steps was unexpected in size." << std::endl;
    return false;
  }
  return true;
}
void Monopod::hold_position() {

  assertm(initialized(), "Requires monopod_sdk is initialized.");
  assertm(current_state_ == MonopodState::RUNNING,
          "Monopod must be in the state [MonopodState::RUNNING] before holding "
          "current position.");

  // Disable limits to avoid triggering the safemode.
  pause_safety_loop = true;
  // Make sure we are in a reset state before going to zero.
  board_->reset();
  leg_->start_holding_loop();
  // Reset here pauses the motors again.
  board_->reset();
  // start limit loop again if it was active before
  pause_safety_loop = false;
}

bool Monopod::is_hold_position() {
  assertm(initialized(), "Requires monopod_sdk is initialized.");
  return motor_joint_indexing.empty() ? false : leg_->is_hold_current_pos();
}

void Monopod::stop_hold_position() {
  assertm(initialized(), "Requires monopod_sdk is initialized.");

  if (current_state_ != MonopodState::HOLDING) {
    // Nothing to stop
    return;
  }

  leg_->stop_hold_current_pos();
  current_state_ = MonopodState::RUNNING;
}

void Monopod::start_safety_loop() {
  assertm(initialized(), "Requires monopod_sdk is initialized.");

  if (safety_loop_running) {
    std::cerr << "Monopod::start_safety_loop(): Safety loop is already active, "
                 "nothing was updated."
              << std::endl;
    return;
  }
  safety_loop_running = true;
  rt_printf("Starting realtime safety loop to ensure physical limits of robot "
            "stay within a safety margin. \n");
  rt_thread_safety_.create_realtime_thread(&Monopod::safety_loop, this);
}

void Monopod::calibrate(const double &hip_home_offset_rad,
                        const double &knee_home_offset_rad) {

  // todo: Make calibration more robust??
  // todo: Update zero for none motor joints. Right now we can just use reset
  // button to get new zero when in physical spot.

  // Check to make sure that the sdk is initialized.
  assertm(initialized(), "Requires monopod_sdk is initialized.");
  // todo: make sure this is worth terminating for.
  assertm(current_state_ == MonopodState::RUNNING,
          "Not in state [MonopodState::RUNNING]. This could mean the robot is "
          "HOLDING or in READ_ONLY mode with no active motors.");

  // Disable limits to avoid triggering the safemode.
  pause_safety_loop = true;
  // Make sure we are in a reset state before going to zero.
  board_->reset();
  bool status = leg_->calibrate(hip_home_offset_rad, knee_home_offset_rad);
  // If status fail cout a error or warning.
  if (!status) {
    std::cerr << "Monopod::calibrate(): [Warn] Failed to reach desired final "
                 "location during calibration."
              << std::endl;
  }
  // Reset here pauses the motors again.
  board_->reset();
  // start limit loop again if it was active before
  pause_safety_loop = false;
}

bool Monopod::is_joint_controllable(const int joint_index) {
  assertm(initialized(), "Requires monopod_sdk is initialized.");
  return Contains(motor_joint_indexing, joint_index);
}

// ========================================
// Getters
// ========================================

std::string Monopod::get_model_name() const { return "monopod"; }

std::unordered_map<std::string, int> Monopod::get_joint_names() const {
  assertm(initialized(), "Requires monopod_sdk is initialized.");

  std::unordered_map<std::string, int> joint_names_;
  for (auto const &pair : joint_names) {
    if (Contains(encoder_joint_indexing, pair.second) ||
        Contains(motor_joint_indexing, pair.second)) {
      joint_names_[pair.first] = pair.second;
    }
  }

  return joint_names_;
}

std::optional<PID> Monopod::get_pid(const int &joint_index) const {

  assertm(false, "NotImplementedError.");

  assertm(initialized(), "Requires monopod_sdk is initialized.");
  if (Contains(motor_joint_indexing, joint_index)) {
    // todo Implement PID read/write
  }

  return std::nullopt;
}

std::optional<JointLimit>
Monopod::get_joint_position_limit(const int &joint_index) const {
  assertm(initialized(), "Requires monopod_sdk is initialized.");
  if (Contains(encoder_joint_indexing, joint_index)) {
    return encoders_.at(joint_index)->get_limit(position);
  }
  return std::nullopt;
}

std::optional<JointLimit>
Monopod::get_joint_velocity_limit(const int &joint_index) const {
  assertm(initialized(), "Requires monopod_sdk is initialized.");
  if (Contains(encoder_joint_indexing, joint_index)) {
    return encoders_.at(joint_index)->get_limit(velocity);
  }
  return std::nullopt;
}

std::optional<JointLimit>
Monopod::get_joint_acceleration_limit(const int &joint_index) const {
  assertm(initialized(), "Requires monopod_sdk is initialized.");
  if (Contains(encoder_joint_indexing, joint_index)) {
    return encoders_.at(joint_index)->get_limit(acceleration);
  }
  return std::nullopt;
}

std::optional<double> Monopod::get_torque_target(const int &joint_index) const {
  assertm(initialized(), "Requires monopod_sdk is initialized.");
  if (Contains(motor_joint_indexing, joint_index)) {
    return motors_.at(joint_index)->get_measured_torque();
  }
  return std::nullopt;
}

std::optional<Vector<double>>
Monopod::get_torque_targets(const Vector<int> &joint_indexes) const {
  assertm(initialized(), "Requires monopod_sdk is initialized.");
  assertm(current_state_ != MonopodState::READ_ONLY,
          "No torque target set in READ_ONLY mode (no active motors).");
  const Vector<int> &jointSerialization =
      joint_indexes.empty() ? motor_joint_indexing : joint_indexes;

  Vector<double> data;
  data.reserve(jointSerialization.size());
  for (auto &joint_index : jointSerialization) {
    if (Contains(motor_joint_indexing, joint_index)) {
      // note: maybe we cna call the single version here.
      data.push_back(motors_.at(joint_index)->get_measured_torque());
      continue;
    } else {
      return std::nullopt;
    }
  }
  return data;
}

std::optional<double> Monopod::get_position(const int &joint_index) const {
  assertm(initialized(), "Requires monopod_sdk is initialized.");
  if (Contains(encoder_joint_indexing, joint_index)) {
    return encoders_.at(joint_index)->get_measured_angle();
  }
  return std::nullopt;
}

std::optional<double> Monopod::get_velocity(const int &joint_index) const {
  assertm(initialized(), "Requires monopod_sdk is initialized.");
  if (Contains(encoder_joint_indexing, joint_index)) {
    return encoders_.at(joint_index)->get_measured_velocity();
  }
  return std::nullopt;
}

std::optional<double> Monopod::get_acceleration(const int &joint_index) const {
  assertm(initialized(), "Requires monopod_sdk is initialized.");
  if (Contains(encoder_joint_indexing, joint_index)) {
    return encoders_.at(joint_index)->get_measured_acceleration();
  }
  return std::nullopt;
}

std::optional<Vector<double>>
Monopod::get_positions(const Vector<int> &joint_indexes) const {
  assertm(initialized(), "Requires monopod_sdk is initialized.");
  auto lambda = [this](int joint_index) -> double {
    return encoders_.at(joint_index)->get_measured_angle();
  };

  return getJointDataSerialized(this, joint_indexes, lambda);
}

std::optional<Vector<double>>
Monopod::get_velocities(const Vector<int> &joint_indexes) const {
  assertm(initialized(), "Requires monopod_sdk is initialized.");
  auto lambda = [this](int joint_index) -> double {
    return encoders_.at(joint_index)->get_measured_velocity();
  };

  return getJointDataSerialized(this, joint_indexes, lambda);
}

std::optional<Vector<double>>
Monopod::get_accelerations(const Vector<int> &joint_indexes) const {
  assertm(initialized(), "Requires monopod_sdk is initialized.");
  auto lambda = [this](int joint_index) -> double {
    return encoders_.at(joint_index)->get_measured_acceleration();
  };

  return getJointDataSerialized(this, joint_indexes, lambda);
}

std::optional<double>
Monopod::get_max_torque_target(const int &joint_index) const {
  assertm(initialized(), "Requires monopod_sdk is initialized.");
  if (Contains(motor_joint_indexing, joint_index)) {
    return motors_.at(joint_index)->get_max_torque();
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
  assertm(false, "NotImplementedError");

  assertm(initialized(), "Requires monopod_sdk is initialized.");
  if (Contains(motor_joint_indexing, joint_index)) {
    // todo Implement PID read/write
    PID pid(p, i, d);
    // motor_->set_pid(pid);
    return true;
  }

  return false;
}

bool Monopod::set_joint_position_limit(const double &max, const double &min,
                                       const int &joint_index) {
  assertm(initialized(), "Requires monopod_sdk is initialized.");
  if (Contains(encoder_joint_indexing, joint_index)) {
    JointLimit limit(min, max);
    encoders_.at(joint_index)->set_limit(position, limit);
    return true;
  }
  return false;
}

bool Monopod::set_joint_velocity_limit(const double &max, const double &min,
                                       const int &joint_index) {
  assertm(initialized(), "Requires monopod_sdk is initialized.");
  if (Contains(encoder_joint_indexing, joint_index)) {
    JointLimit limit(min, max);
    encoders_.at(joint_index)->set_limit(velocity, limit);
    return true;
  }
  return false;
}

bool Monopod::set_joint_acceleration_limit(const double &max, const double &min,
                                           const int &joint_index) {
  assertm(initialized(), "Requires monopod_sdk is initialized.");
  if (Contains(encoder_joint_indexing, joint_index)) {
    JointLimit limit(min, max);
    encoders_.at(joint_index)->set_limit(acceleration, limit);
    return true;
  }
  return false;
}

bool Monopod::set_max_torque_target(const double &max_torque_target,
                                    const int &joint_index) {
  assertm(initialized(), "Requires monopod_sdk is initialized.");
  if (Contains(motor_joint_indexing, joint_index)) {
    motors_.at(joint_index)->set_max_torque(max_torque_target);
    return true;
  } else if (Contains(encoder_joint_indexing, joint_index)) {
    std::cerr << "Monopod::set_max_torque_target(): [Warn] Attempted to set "
                 "max torque on an encoder joint."
              << std::endl;
    return true;
  }
  return false;
}

bool Monopod::set_torque_target(const double &torque_target,
                                const int joint_index) {
  assertm(initialized(), "Requires monopod_sdk is initialized.");
  assertm(current_state_ == MonopodState::RUNNING,
          "Can not set torque target when not in state "
          "[MonopodState::RUNNING]. This could mean the robot is HOLDING or in "
          "READ_ONLY mode with no active motors.");
  if (Contains(motor_joint_indexing, joint_index)) {
    /* automatically clip torque to max in joint module */
    motors_.at(joint_index)->set_torque(torque_target);
    motors_.at(joint_index)->send_torque();

    return true;
  }
  return false;
}

bool Monopod::set_torque_targets(const Vector<double> &torque_targets,
                                 const Vector<int> &joint_indexes) {
  assertm(initialized(), "Requires monopod_sdk is initialized.");
  assertm(current_state_ == MonopodState::RUNNING,
          "Can not set torque target when not in state "
          "[MonopodState::RUNNING]. This could mean the robot is HOLDING or in "
          "READ_ONLY mode with no active motors.");

  const Vector<int> &jointSerialization =
      joint_indexes.empty() ? motor_joint_indexing : joint_indexes;

  assertm(
      torque_targets.size() != jointSerialization.size(),
      "Size of torque targets did not match the number of specified joints.");

  bool ok = true;

  for (size_t i = 0; i != torque_targets.size(); i++) {
    if (Contains(motor_joint_indexing, jointSerialization[i])) {

      motors_.at(jointSerialization[i])->set_torque(torque_targets[i]);
      motors_.at(jointSerialization[i])->send_torque();

      ok = ok && true;
    } else {
      ok = ok && false;
    }
  }

  return ok;
}

/**
 * @brief This is a 100Hz safety_loop that checks the limits of all joints. This
 * is done to make sure the monopod is not in a vulnerable state. Do not want to
 * break the robot.
 */
void Monopod::safety_loop() {
  real_time_tools::Spinner spinner;
  // Check limits at 100hz
  spinner.set_period(0.01);

  while (safety_loop_running) {
    while (pause_safety_loop) {
      spinner.spin();
    }
    bool in_limits = true;
    for (const auto &joint_index : encoder_joint_indexing) {
      in_limits = in_limits && encoders_.at(joint_index)->check_limits();
    }
    if (!in_limits) {
      // If the state is not in safemode already then enter safemode.
      if (valid()) {
        rt_printf(
            "Monopod::safety_loop(): Robot entered safe mode because a "
            "physical limit was reached. Robot must be reset before it is "
            "valid. \n");
      }
      board_->enter_safemode();
    }
    // spin the RT safety_loop.
    spinner.spin();
  }
}

} // namespace monopod_drivers
