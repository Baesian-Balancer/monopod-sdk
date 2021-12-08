#include "monopod_sdk/monopod.hpp"

namespace monopod_drivers
{

Monopod::Monopod(std::shared_ptr<Leg> leg, std::shared_ptr<Planarizer> planarizer)
    {
        leg_ = leg;
        planarizer_ = planarizer;
    }

Monopod::~Monopod()
{
}

/**================================================================================
 * Getters
 */
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
