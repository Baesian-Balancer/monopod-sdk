/**
 * @file motor_board.cpp
 * @author Felix Widmaier (felix.widmaier@tuebingen.mpg.de)
 * @author Manuel Wuthrich (manuel.wuthrich@gmail.com)
 * @author Maximilien Naveau (maximilien.naveau@gmail.com)
 * @brief This file implements the classes from
 * "monopod_sdk/monopod_drivers/devices/motor_board.hpp"
 * @version 0.1
 * @date 2018-11-26
 *
 * @copyright Copyright (c) 2018
 *
 */

#include "monopod_sdk/monopod_drivers/devices/boards.hpp"

namespace monopod_drivers
{
CanBusControlBoards::CanBusControlBoards(std::shared_ptr<CanBusInterface> can_bus,
                                         const size_t& history_length,
                                         const int& control_timeout_ms)
	: can_bus_(can_bus),
	motors_are_paused_(false),
	control_timeout_ms_(control_timeout_ms)
{
	measurement_ = create_vector_of_pointers<ScalarTimeseries>(
		measurement_count, history_length);

	status_ = create_vector_of_pointers<StatusTimeseries>(board_count,
	                                                      history_length);

	control_ = create_vector_of_pointers<ScalarTimeseries>(control_count,
	                                                       history_length);

	command_ = std::make_shared<CommandTimeseries>(history_length, 0, false);

	sent_control_ = create_vector_of_pointers<ScalarTimeseries>(control_count,
	                                                            history_length);

	sent_command_ = std::make_shared<CommandTimeseries>(history_length, 0, false);

	is_loop_active_ = true;
	rt_thread_.create_realtime_thread(&CanBusControlBoards::loop, this);
}

CanBusControlBoards::~CanBusControlBoards()
{
	is_loop_active_ = false;
	rt_thread_.join();
	set_command(ControlBoardsCommand(ControlBoardsCommand::IDs::ENABLE_SYS,
	                                 ControlBoardsCommand::Contents::DISABLE));
	send_newest_command();
}

void CanBusControlBoards::send_if_input_changed()
{
	// send command if a new one has been set ----------------------------------
	if (command_->has_changed_since_tag())
	{
		send_newest_command();
	}

	// send controls if a new one has been set ---------------------------------
	bool controls_have_changed = false;

	for (auto control : control_)
	{
		if (control->has_changed_since_tag()) controls_have_changed = true;
	}
	if (controls_have_changed)
	{
		send_newest_controls();
	}
}

void CanBusControlBoards::wait_until_ready()
{
	rt_printf("waiting for boards and motors to be ready \n");
	bool is_ready = false;
	while (!is_ready)
	{
		is_ready = CanBusControlBoards::is_ready();
	}
	rt_printf("board and motors are ready \n");
}

bool CanBusControlBoards::is_ready()
{
	if (status_[motor_board]->length() == 0 ||
	    status_[encoder_board1]->length() == 0 ||
	    status_[encoder_board2]->length() == 0)
	{
		return false;
	}
	else
	{
		return status_[motor_board]->newest_element().is_ready() &&
		       status_[encoder_board1]->newest_element().is_ready() &&
		       status_[encoder_board2]->newest_element().is_ready();
	}

}

void CanBusControlBoards::pause_motors()
{
	set_control(0, current_target_0);
	set_control(0, current_target_1);
	send_newest_controls();

	set_command(ControlBoardsCommand(ControlBoardsCommand::IDs::SET_CAN_RECV_TIMEOUT,
	                                 ControlBoardsCommand::Contents::DISABLE));
	send_newest_command();

	motors_are_paused_ = true;
}

void CanBusControlBoards::disable_can_recv_timeout()
{
	set_command(ControlBoardsCommand(ControlBoardsCommand::IDs::SET_CAN_RECV_TIMEOUT,
	                                 ControlBoardsCommand::Contents::DISABLE));
	send_newest_command();
}

void CanBusControlBoards::send_newest_controls()
{
	if (motors_are_paused_)
	{
		set_command(ControlBoardsCommand(
						ControlBoardsCommand::IDs::SET_CAN_RECV_TIMEOUT, control_timeout_ms_));
		send_newest_command();
		motors_are_paused_ = false;
	}

	std::array<double, 2> controls;
	for (size_t i = 0; i < control_.size(); i++)
	{
		if (control_[i]->length() == 0)
		{
			rt_printf(
				"you tried to send control but no control has been set\n");
			exit(-1);
		}

		Index timeindex = control_[i]->newest_timeindex();
		controls[i] = (*control_[i])[timeindex];
		control_[i]->tag(timeindex);

		sent_control_[i]->append(controls[i]);
	}

	float current_mtr1 = controls[0];
	float current_mtr2 = controls[1];

	uint8_t data[8];
	uint32_t q_current1, q_current2;

	// Convert floats to Q24 values
	q_current1 = float_to_q24(current_mtr1);
	q_current2 = float_to_q24(current_mtr2);

	// Motor 1
	data[0] = (q_current1 >> 24) & 0xFF;
	data[1] = (q_current1 >> 16) & 0xFF;
	data[2] = (q_current1 >> 8) & 0xFF;
	data[3] = q_current1 & 0xFF;

	// Motor 2
	data[4] = (q_current2 >> 24) & 0xFF;
	data[5] = (q_current2 >> 16) & 0xFF;
	data[6] = (q_current2 >> 8) & 0xFF;
	data[7] = q_current2 & 0xFF;

	CanBusFrame can_frame;
	can_frame.id = CanframeIDs::IqRef;
	for (size_t i = 0; i < 7; i++)
	{
		can_frame.data[i] = data[i];
	}
	can_frame.dlc = 8;

	can_bus_->set_input_frame(can_frame);
	can_bus_->send_if_input_changed();
}

void CanBusControlBoards::send_newest_command()
{
	if (command_->length() == 0)
	{
		rt_printf("you tried to send command but no command has been set\n");
		exit(-1);
	}

	Index timeindex = command_->newest_timeindex();
	ControlBoardsCommand command = (*command_)[timeindex];
	command_->tag(timeindex);
	sent_command_->append(command);

	uint32_t id = command.id_;
	int32_t content = command.content_;

	uint8_t data[8];

	// content
	data[0] = (content >> 24) & 0xFF;
	data[1] = (content >> 16) & 0xFF;
	data[2] = (content >> 8) & 0xFF;
	data[3] = content & 0xFF;

	// command
	data[4] = (id >> 24) & 0xFF;
	data[5] = (id >> 16) & 0xFF;
	data[6] = (id >> 8) & 0xFF;
	data[7] = id & 0xFF;

	CanBusFrame can_frame;
	can_frame.id = CanframeIDs::COMMAND_ID;
	for (size_t i = 0; i < 8; i++)
	{
		can_frame.data[i] = data[i];
	}
	can_frame.dlc = 8;

	can_bus_->set_input_frame(can_frame);
	can_bus_->send_if_input_changed();
}

void CanBusControlBoards::loop()
{
	pause_motors();

	// initialize board --------------------------------------------------------
	set_command(ControlBoardsCommand(ControlBoardsCommand::IDs::ENABLE_SYS,
	                                 ControlBoardsCommand::Contents::ENABLE));
	send_newest_command();

	set_command(ControlBoardsCommand(ControlBoardsCommand::IDs::SEND_ALL,
	                                 ControlBoardsCommand::Contents::ENABLE));
	send_newest_command();

	set_command(ControlBoardsCommand(ControlBoardsCommand::IDs::ENABLE_MTR1,
	                                 ControlBoardsCommand::Contents::ENABLE));
	send_newest_command();

	set_command(ControlBoardsCommand(ControlBoardsCommand::IDs::ENABLE_MTR2,
	                                 ControlBoardsCommand::Contents::ENABLE));
	send_newest_command();

	// receive data from board in a loop ---------------------------------------
	long int timeindex = can_bus_->get_output_frame()->newest_timeindex();
	while (is_loop_active_)
	{
		CanBusFrame can_frame;
		Index received_timeindex = timeindex;
		can_frame = (*can_bus_->get_output_frame())[received_timeindex];

		if (received_timeindex != timeindex)
		{
			rt_printf(
				"did not get the timeindex we expected! "
				"received_timeindex: %d, "
				"desired_timeindex: %d\n",
				int(received_timeindex),
				int(timeindex));
			exit(-1);
		}

		timeindex++;


		// convert to measurement ------------------------------------------
		double measurement_0 = qbytes_to_float(can_frame.data.begin());
		double measurement_1 = qbytes_to_float((can_frame.data.begin() + 4));

		switch (can_frame.id)
		{
		case CanframeIDs::Iq:
			measurement_[current_0]->append(measurement_0);
			measurement_[current_1]->append(measurement_1);
			break;

		case CanframeIDs::BOARD1_POS:
			// Convert the position unit from the blmc card (kilo-rotations)
			// into rad.
			measurement_[position_0]->append(measurement_0 * 2 * M_PI);
			measurement_[position_1]->append(measurement_1 * 2 * M_PI);
			break;

		case CanframeIDs::BOARD2_POS:
			// Convert the position unit from the blmc card (kilo-rotations)
			// into rad.
			measurement_[position_2]->append(measurement_0 * 2 * M_PI);
			measurement_[position_3]->append(measurement_1 * 2 * M_PI);
			break;

		case CanframeIDs::BOARD3_POS:
			// Convert the position unit from the blmc card (kilo-rotations)
			// into rad.
			measurement_[position_4]->append(measurement_0 * 2 * M_PI);
			break;

		case CanframeIDs::BOARD1_VEL:
			// Convert the speed unit from the blmc card
			// (kilo-rotations-per-minutes) into rad/s.
			measurement_[velocity_0]->append(measurement_0 * 2 * M_PI *
			                                 (1000. / 60.));
			measurement_[velocity_1]->append(measurement_1 * 2 * M_PI *
			                                 (1000. / 60.));
			break;

		case CanframeIDs::BOARD2_VEL:
			// Convert the speed unit from the blmc card
			// (kilo-rotations-per-minutes) into rad/s.
			measurement_[velocity_2]->append(measurement_0 * 2 * M_PI *
			                                 (1000. / 60.));
			measurement_[velocity_3]->append(measurement_1 * 2 * M_PI *
			                                 (1000. / 60.));
			break;

		case CanframeIDs::BOARD3_VEL:
			// Convert the speed unit from the blmc card
			// (kilo-rotations-per-minutes) into rad/s.
			measurement_[velocity_4]->append(measurement_0 * 2 * M_PI *
			                                 (1000. / 60.));
			break;

		case CanframeIDs::BOARD1_ACC:
			// Convert the speed unit from the blmc card
			// (kilo-rotations-per-minutes) into rad/s.
			measurement_[acceleration_0]->append(measurement_0 * 2 * M_PI *
			                                     (1000. / 60.));
			measurement_[acceleration_1]->append(measurement_1 * 2 * M_PI *
			                                     (1000. / 60.));
			break;

		case CanframeIDs::BOARD2_ACC:
			// Convert the speed unit from the blmc card
			// (kilo-rotations-per-minutes) into rad/s.
			measurement_[acceleration_2]->append(measurement_0 * 2 * M_PI *
			                                     (1000. / 60.));
			measurement_[acceleration_3]->append(measurement_1 * 2 * M_PI *
			                                     (1000. / 60.));
			break;

		case CanframeIDs::BOARD3_ACC:
			// Convert the speed unit from the blmc card
			// (kilo-rotations-per-minutes) into rad/s.
			measurement_[acceleration_4]->append(measurement_0 * 2 * M_PI *
			                                     (1000. / 60.));
			break;

		case CanframeIDs::ADC6:
			measurement_[analog_0]->append(measurement_0);
			measurement_[analog_1]->append(measurement_1);
			break;

		case CanframeIDs::BOARD1_ENC_INDEX:
		{
			// here the interpretation of the message is different,
			// we get a motor index and a measurement
			uint8_t motor_index = can_frame.data[4];
			if (motor_index == 0)
			{
				measurement_[encoder_index_0]->append(measurement_0 * 2 *
				                                      M_PI);
			}
			else if (motor_index == 1)
			{
				measurement_[encoder_index_1]->append(measurement_0 * 2 *
				                                      M_PI);
			}
			else
			{
				rt_printf(
					"ERROR: Invalid motor number"
					"for encoder index: %d\n",
					motor_index);
				exit(-1);
			}
			break;
		}

		case CanframeIDs::BOARD1_STATUSMSG:
		{
			MotorBoardStatus status;
			uint8_t data = can_frame.data[0];
			status.system_enabled = data >> 0;
			status.motor1_enabled = data >> 1;
			status.motor1_ready = data >> 2;
			status.motor2_enabled = data >> 3;
			status.motor2_ready = data >> 4;
			status.error_code = data >> 5;

			status_[motor_board]->append(status);
			break;
		}

		case CanframeIDs::BOARD2_STATUSMSG:
		{
			EncoderBoardStatus status;
			uint8_t data = 00000000;
			status.error_code = data >> 5;
			status_[encoder_board1]->append(status);
			break;
		}

		case CanframeIDs::BOARD3_STATUSMSG:
		{
			EncoderBoardStatus status;
			uint8_t data = 00000000;
			status.error_code = data >> 5;
			status_[encoder_board2]->append(status);
			break;
		}
		}

		//        static int count = 0;
		//        if(count % 4000 == 0)
		//        {
		//            print_status();
		//        }
		//        count++;
	}
}

void CanBusControlBoards::print_status()
{
	rt_printf("ouptus ======================================\n");
	rt_printf("measurements: -------------------------------\n");
	for (size_t i = 0; i < measurement_.size(); i++)
	{
		rt_printf("%d: ---------------------------------\n", int(i));
		if (measurement_[i]->length() > 0)
		{
			double measurement = measurement_[i]->newest_element();
			rt_printf("value %f:\n", measurement);
		}
	}

	rt_printf("status: ---------------------------------\n");
	if (status_[motor_board]->length() > 0) status_[motor_board]->newest_element().print();
	if (status_[encoder_board1]->length() > 0) status_[encoder_board1]->newest_element().print();
	if (status_[encoder_board2]->length() > 0) status_[encoder_board2]->newest_element().print();

	//        rt_printf("inputs ======================================\n");

	//        for(size_t i = 0; i < control_names.size(); i++)
	//        {
	//            rt_printf("%s: ---------------------------------\n",
	//                                 control_names[i].c_str());
	//            if(control_.at(control_names[i])->length() > 0)
	//            {
	//                double control =
	//                        control_.at(control_names[i])->newest_element();
	//                rt_printf("value %f:\n", control);
	//            }
	//        }

	//        rt_printf("command: ---------------------------------\n");
	//        if(command_[command]->length() > 0)
	//            command_[command]->newest_element().print();
}

}  // namespace monopod_drivers
