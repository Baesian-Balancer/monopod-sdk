#include "monopod_sdk/monopod.hpp"

namespace monopod_drivers
{

Monopod::Monopod(std::shared_ptr<Leg> leg, std::shared_ptr<Planarizer> planarizer)
{
    leg_ = leg;
    planarizer_ = planarizer;
}

Monopod::Monopod()
{
    // Initialize the communication with the can bus.
    //
    // NOTE: Default is Leg uses can0 and Planarizer uses can1 and can2
    auto can_bus1 = std::make_shared<blmc_drivers::CanBus>("can0");
    auto can_bus2 = std::make_shared<blmc_drivers::CanBus>("can1");
    auto can_bus3 = std::make_shared<blmc_drivers::CanBus>("can2");

    auto motor_board1 =
        std::make_shared<blmc_drivers::CanBusMotorBoard>(can_bus1);
    auto motor_board2 =
        std::make_shared<blmc_drivers::CanBusMotorBoard>(can_bus2);
    auto motor_board3=
        std::make_shared<blmc_drivers::CanBusMotorBoard>(can_bus3);
    
    // Connect leg motors to motor_board1
    auto motor_hip = std::make_shared<blmc_drivers::SafeMotor>(motor_board1, 0);
    auto motor_knee = std::make_shared<blmc_drivers::SafeMotor>(motor_board1, 1);

    // NOTE: By default boom yaw and boom pitch encoders are connected to board 2,
    // boom connector to board 3
    auto encoder_by =
        std::make_shared<monopod_drivers::Encoder>(motor_board2, 0);
    auto encoder_bp =
        std::make_shared<monopod_drivers::Encoder>(motor_board2, 1);
    auto encoder_bc = 
        std::make_shared<monopod_drivers::Encoder>(motor_board3, 0);


    auto leg = std::make_shared<Leg>(motor_hip, motor_knee);
    auto planarizer = std::make_shared<Planarizer>(encoder_by, encoder_bp, encoder_bc);

    leg_ = leg;
    planarizer_ = planarizer;
}

Monopod::~Monopod()
{
}

/**================================================================================
 * Getters
 */

/**
 * @brief Returns all the data in an unordered map. Key is a joint
 * in joint_str_indexer, value is a vector containing pos, vel, accel, torq,
 * in that order
 * 
 * @return std::unordered_map<std::string, std::vector<double>> 
 */
std::unordered_map<std::string, std::vector<double>> Monopod::get_measurements()
{
    std::unordered_map<std::string, std::vector<double>> joint_dataframe;

    for(int i = 0; i < joint_str_indexer.size(); i++)
    {
        std::vector<double> joint_data;
        std::string joint_string = joint_str_indexer[i];

        // get all data
        //
        // TODO: implement accel
        if (i == hip || i == knee)
        {
            double pos = leg_->get_measurement(i, position)->newest_element();
            double vel = leg_->get_measurement(i, velocity)->newest_element();
            double curr = leg_->get_measurement(i, current)->newest_element();
            joint_data.insert(joint_data.end(), {pos, vel, curr});
        }
        else
        {
            double pos = planarizer_->get_measurement(i, position)->newest_element();
            double vel = planarizer_->get_measurement(i, velocity)->newest_element();
            joint_data.insert(joint_data.end(), {pos, vel});
        }  
        joint_dataframe[joint_string] = joint_data;  
    }
    return joint_dataframe;
}

Monopod::ReturnValueStatus Monopod::get_position(const int joint_index)
{
    // if joint_index is equal to hip or knee use leg, if equal to boom_connector
    // boom_yaw or boom_pitch use planarizer
    switch(joint_index)
    {
        case hip:
            return_value_status_.valid = true;
            return_value_status_.value_series = leg_->get_measurement(joint_index, position)->newest_element();
            return return_value_status_;
        case knee:
            return_value_status_.valid = true;
            return_value_status_.value_series = leg_->get_measurement(joint_index, position)->newest_element();
            return return_value_status_;
        case boom_connector:
            return_value_status_.valid = true;
            return_value_status_.value_series = planarizer_->get_measurement(joint_index, position)->newest_element();
            return return_value_status_;
        case boom_yaw:
            return_value_status_.valid = true;
            return_value_status_.value_series = planarizer_->get_measurement(joint_index, position)->newest_element();
            return return_value_status_;
        case boom_pitch:
            return_value_status_.valid = true;
            return_value_status_.value_series = planarizer_->get_measurement(joint_index, position)->newest_element();
            return return_value_status_;
        default:
            std::cout<<"Passed joint index not valid - not part of monopod" << std::endl;
            return_value_status_.valid = false;
            return_value_status_.value_series = NAN;
            return return_value_status_;
    }
}

Monopod::ReturnValueStatus Monopod::get_velocity(const int joint_index)
{
    // if joint_index is equal to hip or knee use leg, if equal to boom_connector
    // boom_yaw or boom_pitch use planarizer
    switch(joint_index)
    {
        case hip:
            return_value_status_.valid = true;
            return_value_status_.value_series = leg_->get_measurement(joint_index, velocity)->newest_element();
            return return_value_status_;
        case knee:
            return_value_status_.valid = true;
            return_value_status_.value_series = leg_->get_measurement(joint_index, velocity)->newest_element();
            return return_value_status_;
        case boom_connector:
            return_value_status_.valid = true;
            return_value_status_.value_series = planarizer_->get_measurement(joint_index, velocity)->newest_element();
            return return_value_status_;
        case boom_yaw:
            return_value_status_.valid = true;
            return_value_status_.value_series = planarizer_->get_measurement(joint_index, velocity)->newest_element();
            return return_value_status_;
        case boom_pitch:
            return_value_status_.valid = true;
            return_value_status_.value_series = planarizer_->get_measurement(joint_index, velocity)->newest_element();
            return return_value_status_;
        default:
            std::cout<<"Passed joint index not valid - not part of monopod" << std::endl;
            return_value_status_.valid = false;
            return_value_status_.value_series = NAN;
            return return_value_status_;
    }
}


Monopod::ReturnValueStatus Monopod::get_current(const int joint_index)
{
    // if joint_index is equal to hip or knee use leg, if equal to boom_connector
    // boom_yaw or boom_pitch use planarizer
    switch(joint_index)
    {
        case hip:
            return_value_status_.valid = true;
            return_value_status_.value_series = leg_->get_measurement(joint_index, current)->newest_element();
            return return_value_status_;
        case knee:
            return_value_status_.valid = true;
            return_value_status_.value_series = leg_->get_measurement(joint_index, current)->newest_element();
            return return_value_status_;
        default:
            std::cout<<"Non-valid joint index for current - must be hip or knee" << std::endl;
            return_value_status_.valid = false;
            return_value_status_.value_series = NAN;
            return return_value_status_;
    }
}


Monopod::PID Monopod::get_PID()
{
    return pid_;
}

std::vector<std::string> Monopod::get_joint_indexing() const
{
    return joint_str_indexer;
}


/**=======================================================================
 * Setters
*/


Monopod::ReturnValueStatus Monopod::set_target_torque(const int joint_index, const double &torque_target)
{
    switch(joint_index)
    {
        case hip:
            leg_->motors_[hip]->set_current_target(torque_target);
            leg_->motors_[hip]->send_if_input_changed();
            return_value_status_.valid = true;
            return_value_status_.value_series = torque_target;
            return return_value_status_;
        case knee:
            leg_->motors_[knee]->set_current_target(torque_target);
            leg_->motors_[knee]->send_if_input_changed();
            return_value_status_.valid = true;
            return_value_status_.value_series = torque_target;
            return return_value_status_;
        default:
            std::cout<<"Joint index not valid - must be hip or knee" << std::endl;
            return_value_status_.valid = false;
            return_value_status_.value_series = NAN;
            return return_value_status_;


    }
}

void Monopod::set_PID(const double &p_value, const double &i_value, const double &d_value)
{
    pid_.p = p_value;
    pid_.i = i_value;
    pid_.d = d_value;
}


} // end monopod_drivers namespace
