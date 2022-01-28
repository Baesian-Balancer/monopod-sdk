/**
 * @file sine_position_control.cpp
 * @copyright Copyright (c) 2018-2020, New York University and Max Planck
 * Gesellschaft, License BSD-3-Clause
 */

#include "sine_position_control.hpp"
#include "real_time_tools/spinner.hpp"
#include "real_time_tools/timer.hpp"
#include <fstream>

namespace monopod_drivers {

void SinePositionControl::loop() {
  // here is the control in torque (NM)
  double actual_torque_hip = 0.0;
  double actual_torque_knee = 0.0;

  double actual_position_hip = 0.0;
  double actual_position_knee = 0.0;

  double actual_velocity_hip = 0.0;
  double actual_velocity_knee = 0.0;

  double local_time = 0.0;
  double control_period = 0.001;

  // sine torque params
  double amplitude = 0 /*3.1415*/;
  double frequence = 0.5;

  // here is the control in torque (NM)
  double desired_torque_hip = 0.0;
  double desired_torque_knee = 0.0;

  double desired_position_hip = 0.0;
  double desired_position_knee = 0.0;

  double desired_velocity_hip = 0.0;
  double desired_velocity_knee = 0.0;

  double kp, kd;
  kp = 1.0;
  kd = 0;
  set_gains(kp, kd);

  real_time_tools::Spinner spinner;
  spinner.set_period(control_period); // here we spin every 1ms
  real_time_tools::Timer time_logger;
  size_t count = 0;
  while (!stop_loop_) {
    time_logger.tic();
    local_time = count * control_period;

    std::vector<double> data =
        sdk_->get_positions({hip_joint, knee_joint}).value();
    // compute the control
    actual_position_hip = data[0];
    actual_position_knee = data[1];

    data = sdk_->get_velocities({hip_joint, knee_joint}).value();
    actual_velocity_hip = data[0];
    actual_velocity_knee = data[1];

    data = sdk_->get_torque_targets({hip_joint, knee_joint}).value();
    actual_torque_hip = data[0];
    actual_torque_knee = data[1];

    double desired_position =
        amplitude * sin(2 * M_PI * frequence * local_time);
    desired_position_hip = desired_position;
    desired_position_knee = -desired_position; // opposite to make the bend
                                               // create vertical movement.

    desired_torque_hip = kp_ * (desired_position_hip - actual_position_hip) +
                         kd_ * (desired_velocity_hip - actual_velocity_hip);

    desired_torque_knee = kp_ * (desired_position_knee - actual_position_knee) +
                          kd_ * (desired_velocity_knee - actual_velocity_knee);

    sdk_->set_torque_targets({desired_torque_hip, desired_torque_knee},
                             {hip_joint, knee_joint});

    encoders_[0].push_back(actual_position_hip);
    velocities_[0].push_back(actual_velocity_hip);
    torques_[0].push_back(actual_torque_hip);
    control_buffer_[0].push_back(desired_torque_hip);

    encoders_[1].push_back(actual_position_knee);
    velocities_[1].push_back(actual_velocity_knee);
    torques_[1].push_back(actual_torque_knee);
    control_buffer_[1].push_back(desired_torque_knee);

    // we sleep here 1ms.
    spinner.spin();
    // measure the time spent.
    time_logger.tac();

    // Printings
    if ((count % (int)(0.2 / control_period)) == 0) {
      rt_printf("\33[H\33[2J"); // clear screen
      for (unsigned int i = 0; i < NUMBER_LEG_JOINTS; ++i) {
        rt_printf("des_pose: %8f ; ", desired_position);
        rt_printf("des_torque_hip: %8f ; ", desired_torque_hip);
        rt_printf("des_torque_knee: %8f ; ", desired_torque_knee);
        sdk_->print();
      }
      time_logger.print_statistics();
      fflush(stdout);
    }
    ++count;

  } // endwhile
  time_logger.dump_measurements("/tmp/demo_pd_control_time_measurement");
}

void SinePositionControl::stop_loop() {
  // dumping stuff
  std::string file_name = "/tmp/sine_position_xp.dat";
  try {
    std::ofstream log_file(file_name, std::ios::binary | std::ios::out);
    log_file.precision(10);

    assert(encoders_[0].size() == velocities_[0].size() &&
           velocities_[0].size() == control_buffer_[0].size() &&
           control_buffer_[0].size() == torques_[0].size());
    for (size_t j = 0; j < encoders_[0].size(); ++j) {
      for (size_t i = 0; i < encoders_.size(); ++i) {
        log_file << encoders_[i][j] << " " << velocities_[i][j] << " "
                 << control_buffer_[i][j] << " " << torques_[i][j] << " ";
      }
      log_file << std::endl;
    }

    log_file.flush();
    log_file.close();
  } catch (...) {
    rt_printf("fstream Error in dump_tic_tac_measurements(): "
              "no time measurment saved\n");
  }

  rt_printf("dumped the trajectory");
}

} // namespace monopod_drivers
