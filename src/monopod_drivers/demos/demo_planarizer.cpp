/**
 * @file demo_1_motor_print_everything.cpp
 * @copyright Copyright (c) 2018-2020, New York University and Max Planck
 * Gesellschaft, License BSD-3-Clause
 */

#include <tuple>

#include <monopod_sdk/blmc_drivers/devices/can_bus.hpp>
#include <monopod_sdk/blmc_drivers/devices/motor.hpp>
#include <monopod_sdk/blmc_drivers/devices/motor_board.hpp>

typedef std::tuple<std::shared_ptr<blmc_drivers::MotorInterface>,
                   std::shared_ptr<blmc_drivers::AnalogSensorInterface>>
    MotorAndSlider;

struct Hardware
{
    std::shared_ptr<blmc_drivers::CanBusInterface> can_bus;
    std::shared_ptr<blmc_drivers::MotorBoardInterface> motor_board;
    std::shared_ptr<blmc_drivers::MotorInterface> motor;
    std::shared_ptr<blmc_drivers::AnalogSensorInterface> slider;
};


static THREAD_FUNCTION_RETURN_TYPE printing_loop(void* canbus_ptr)
{
    // cast input arguments to the right format --------------------------------
    blmc_drivers::CanBusInterface& canbus = *(static_cast<blmc_drivers::CanBusInterface*>(canbus_ptr));

    // print info --------------------------------------------------------------
    long int timeindex =
       canbus->get_output_frame()->newest_timeindex();

    while (true)
    {
        long int received_timeindex = timeindex;
        // this will return the element with the index received_timeindex,
        // if this element does not exist anymore, it will return the oldest
        // element it still has and change received_timeindex to the appropriate
        // index.
        blmc_drivers::CanBusFrame can_frame =
            can_bus->get_output_frame()[received_timeindex];
        timeindex++;

        rt_printf("timeindex: %ld\n", timeindex);
        can_frame.print();
    }
    return THREAD_FUNCTION_RETURN_VALUE;
}

int main(int, char**)
{
    // First of all one need to initialize the communication with the can bus.
    auto can_bus1 = std::make_shared<blmc_drivers::CanBus>("can0");
    auto can_bus2 = std::make_shared<blmc_drivers::CanBus>("can1");

    // Then we create a motor board object that will use the can bus in order
    // communicate between this application and the actual motor board.
    // Important: the blmc motors are alinged during this stage.
    auto motor_board1 =
        std::make_shared<blmc_drivers::CanBusMotorBoard>(can_bus1);
    auto motor_board2 =
        std::make_shared<blmc_drivers::CanBusMotorBoard>(can_bus2);

    // create the motor object that have an index that define the port on which
    // they are plugged on the motor board. This object takes also a MotorBoard
    // object to be able to get the sensors and send the control consistantly.
    // These safe motors have the ability to bound the current that is given
    // as input.
    auto encoder_by =
        std::make_shared<Encoder>(motor_board1, 0);
    auto encoder_bp =
        std::make_shared<Encoder>(motor_board1, 1);
    auto encoder_bc = 
        std::make_shared<Encoder>(motor_board2, 0);
    
    auto planarizer = std::make_shared<Planarizer>(encoder_by, encoder_bp, encoder_bc)

    // // start real-time control loop --------------------------------------------
    // real_time_tools::RealTimeThread control_thread;
    // control_thread.create_realtime_thread(&control_loop, &hardware);

    // start real-time printing loop -------------------------------------------
    real_time_tools::RealTimeThread printing_thread;
    printing_thread.create_realtime_thread(&printing_loop, &canbus1);

    rt_printf("control loop started \n");
    // control_thread.join();
    printing_thread.join();
    return 0;
}
