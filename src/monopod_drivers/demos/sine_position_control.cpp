/**
 * @file sine_position_control.cpp
 * @copyright Copyright (c) 2018-2020, New York University and Max Planck
 * Gesellschaft, License BSD-3-Clause
 */

#include "sine_position_control.hpp"
#include <fstream>
#include "real_time_tools/spinner.hpp"
#include "real_time_tools/timer.hpp"

namespace monopod_drivers
{

void SinePositionControl::loop()
{
    const int& position_index = monopod_drivers::Leg::MotorMeasurementIndexing::position;
    const int& velocity_index = monopod_drivers::Leg::MotorMeasurementIndexing::velocity;
    const int& current_index = monopod_drivers::Leg::MotorMeasurementIndexing::current;
    // some data
    
    Vector actual_position(0.0, 0.0);
    Vector actual_velocity(0.0, 0.0);
    Vector actual_current(0.0, 0.0);
    double local_time = 0.0;
    double control_period = 0.001;
    // sine torque params
    double amplitude = 0.1 /*3.1415*/;
    double frequence = 0.5;
    // here is the control in current (Ampere)
    double desired_position = 0.0;
    double desired_velocity = 0.0;
    Vector desired_torque;

    Vector desired_pos;
    Vector desired_vel;

    real_time_tools::Spinner spinner;
    spinner.set_period(control_period);  // here we spin every 1ms
    real_time_tools::Timer time_logger;
    size_t count = 0;
    while (!stop_loop_)
    {
        time_logger.tic();
        local_time = count * control_period;

        // compute the control
        actual_position = leg_->get_measurements(position_index);

        actual_velocity = leg_->get_measurements(velocity_index);
        
        actual_current = leg_->get_measurements(current_index);

        desired_position =
            amplitude * sin(2 * M_PI * frequence * local_time);
        desired_velocity = 0.0; /* 2 * M_PI * frequence * amplitude *
                            cos(2 * M_PI * frequence * local_time)*/
        
        desired_pos[0] = desired_position;
        desired_pos[1] = desired_position;
        desired_vel[0] = desired_velocity;
        desired_vel[1] = desired_velocity;

        desired_torque = kp_ * (desired_pos - actual_position) +
                              kd_ * (desired_vel - actual_velocity);
        
        leg_->set_target_torques(desired_torque);
        leg_->send_target_torques();
        
        
        // for (size_t i = 0; i < leg_->motors_.size(); ++i)
        // {
        //     actual_position = leg_->get_measurement(i, blmc_position_index)
        //                           ->newest_element();
        //     actual_velocity = leg_->get_measurement(i, blmc_velocity_index)
        //                           ->newest_element();
        //     actual_current = leg_->get_measurement(i, blmc_current_index)
        //                          ->newest_element();

        //     desired_position =
        //         amplitude * sin(2 * M_PI * frequence * local_time);

        //     desired_velocity = 0.0 /* 2 * M_PI * frequence * amplitude *
        //                         cos(2 * M_PI * frequence * local_time)*/
        //         ;
        //     desired_torque = kp_ * (desired_position - actual_position) +
        //                       kd_ * (desired_velocity - actual_velocity);
        //     leg_->set_current_target(desired_torque, i);
        // }
        // Send the controls and log stuff

        for (int i = 0; i < 2; i++)
        {
            encoders_[i].push_back(actual_position[i]);
            velocities_[i].push_back(actual_velocity[i]);
            currents_[i].push_back(actual_current[i]);
            control_buffer_[i].push_back(desired_torque[i]);
        }

        // we sleep here 1ms.
        spinner.spin();
        // measure the time spent.
        time_logger.tac();

        // Printings
        if ((count % (int)(0.2 / control_period)) == 0)
        {
            rt_printf("\33[H\33[2J");  // clear screen
            for (unsigned int i = 0; i < leg_->num_joints_; ++i)
            {
                rt_printf("des_pose: %8f ; ", desired_position);
                // leg_->motors_[i]->print();
            }
            time_logger.print_statistics();
            fflush(stdout);
        }
        ++count;

    }  // endwhile
    time_logger.dump_measurements("/tmp/demo_pd_control_time_measurement");
}

void SinePositionControl::stop_loop()
{
    // dumping stuff
    std::string file_name = "/tmp/sine_position_xp.dat";
    try
    {
        std::ofstream log_file(file_name, std::ios::binary | std::ios::out);
        log_file.precision(10);

        assert(encoders_[0].size() == velocities_[0].size() &&
               velocities_[0].size() == control_buffer_[0].size() &&
               control_buffer_[0].size() == currents_[0].size());
        for (size_t j = 0; j < encoders_[0].size(); ++j)
        {
            for (size_t i = 0; i < encoders_.size(); ++i)
            {
                log_file << encoders_[i][j] << " " << velocities_[i][j] << " "
                         << control_buffer_[i][j] << " " << currents_[i][j]
                         << " ";
            }
            log_file << std::endl;
        }

        log_file.flush();
        log_file.close();
    }
    catch (...)
    {
        rt_printf(
            "fstream Error in dump_tic_tac_measurements(): "
            "no time measurment saved\n");
    }

    rt_printf("dumped the trajectory");
}

}  // namespace monopod_drivers
