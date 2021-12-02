#include "monopod.hpp"

namespace monopod_drivers
{


Monopod::Monopod(
                    std::shared_ptr<blmc_drivers::Leg> leg,
                    std::shared_ptr<monopod_drivers::Planarizer> planarizer)
    {
        monopod_[monopod_drivers::Monopod::leg] = leg
        monopod_[monopod_drivers::Monopod::planarizer] = planarizer
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
            // TODO: Rename this? Not clear we are reading encoders with method
            // named get_motor_measurement 
            ReturnValueStatus.valid = true;
            ReturnValueStatus.value_series = monopod_[leg]->get_motor_measurement(joint_index, Monopod::position);
            return ReturnValueStatus
        case knee:
            ReturnValueStatus.valid = true
            ReturnValueStatus.value_series = monopod_[leg]->get_motor_measurement(joint_index, Monopod::position);
            return ReturnValueStatus
        case boom_connector:
            ReturnValueStatus.valid = true;
            ReturnValueStatus.value_series = monopod_[planarizer]->get_measurement(joint_index, Monopod::position);
            return ReturnValueStatus
        case boom_yaw:
            ReturnValueStatus.valid = true;
            ReturnValueStatus.value_series = monopod_[planarizer]->get_measurement(joint_index, Monopod::position);
            return ReturnValueStatus
        case boom_pitch:
            ReturnValueStatus.valid = true;
            ReturnValueStatus.value_series = monopod_[planarizer]->get_measurement(joint_index, Monopod::position);
        default:
            std::cout<<"Passed joint index not valid - not part of monopod" << std::endl;
            ReturnValueStatus.valid = false;
            ReturnValueStatus.value_series = NULL;
            return ReturnValueStatus
    }
}

Monopod::ReturnValueStatus Monopod::get_velocity(const int joint_index)
{
    // if joint_index is equal to hip or knee use leg, if equal to boom_connector
    // boom_yaw or boom_pitch use planarizer
    switch(joint_index)
    {
        case hip:
            // TODO: Rename this? Not clear we are reading encoders with method
            // named get_motor_measurement
            ReturnValueStatus.valid = true;
            ReturnValueStatus.value_series = monopod_[leg]->get_motor_measurement(joint_index, Monopod::velocity);
            return ReturnValueStatus
        case knee:
            ReturnValueStatus.valid = true
            ReturnValueStatus.value_series = monopod_[leg]->get_motor_measurement(joint_index, Monopod::velocity);
            return ReturnValueStatus
        case boom_connector:
            ReturnValueStatus.valid = true;
            ReturnValueStatus.value_series = monopod_[planarizer]->get_measurement(joint_index, Monopod::velocity);
            return ReturnValueStatus
        case boom_yaw:
            ReturnValueStatus.valid = true;
            ReturnValueStatus.value_series = monopod_[planarizer]->get_measurement(joint_index, Monopod::velocity);
            return ReturnValueStatus
        case boom_pitch:
            ReturnValueStatus.valid = true;
            ReturnValueStatus.value_series = monopod_[planarizer]->get_measurement(joint_index, Monopod::velocity);
        default:
            std::cout<<"Passed joint index not valid - not part of monopod" << std::endl;
            ReturnValueStatus.valid = false;
            ReturnValueStatus.value_series = NULL;
            return ReturnValueStatus
    }
}

PID Monopod::get_PID()
{
    return Monopod::PID
}

/**=======================================================================
 * Setters
 */


Monopod::ReturnValueStatus set_target_torque(const int joint_index, const double &torque_target) const
{
    switch(joint_index)
    {
        case hip:
            monopod_[leg]->motors_[hip]->set_current_target(torque_target);
            monopod_[leg]->motors_[hip]->send_if_input_changed();
            ReturnValueStatus.valid = true;
            ReturnValueStatus.value_series = torque_target;
            return ReturnValueStatus
        case knee:
            monopod_[leg]->motors_[knee]->set_current_target(torque_target); 
            monopod_[leg]->motors_[knee]->send_if_input_changed();
            ReturnValueStatus.valid = true;
            ReturnValueStatus.value_series = torque_target;
            return ReturnValueStatus
        default:
            std::cout<<"Joint index not valid - must be hip or knee" << std::endl;
            ReturnValueStatus.valid = false;
            ReturnValueStatus.value_series = NULL;
            return ReturnValueStatus


    }
}

void Monopod::set_PID(const double &p_value, const double &i_value, const double &d_value)
{
    Monopod::PID.p = p_value;
    Monopod::PID.i = i_value;
    Monopod::PID.d = d_value;
}


} // end monopod_drivers namespace