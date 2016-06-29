#include <stdio.h>
#include <thread>

#include "proto_bridge.h"
#include "../protos/messages_immortals_robot_wrapper.pb.h"

#include "utility/netraw.h"

#include "tests.h"

int main()
{
	test_command();
	test_config_control();
	test_config_shoot();
	test_config_on_board();
	test_matrix();
	test_feedback();

	auto send_func = [&]()
	{
		Net::UDP udp;
		udp.open();

		Net::Address dest_address;
		dest_address.setHost("224.5.92.5", 60005);

		int robot_ids[1] = { 0 };

		Immortals::Data::RobotMessageFrame message_frame;

		int mode = 0;

		if (mode == 0)
		{
			for (auto id : robot_ids)
			{
				auto message = message_frame.add_messages();
				message->set_robot_id(id);

				auto command = message->mutable_command();

				command->mutable_velocity()->set_x(-10.0f);
				command->mutable_velocity()->set_y(1.0f);

				command->set_omega(200.0f);
				command->set_target_orientation(0.0f);
				command->set_orientation(0.0f);

				command->set_chip(0.0f);

				command->set_dribbler(0.0f);
				command->set_servo(0.0f);
				command->set_beep(0.0f);

				command->set_feedback(Immortals::Data::RobotCommand_FeedbackRequestType_Debug);
			}
		}
		else if (mode == 1)
		{
			for (auto id : robot_ids)
			{
				auto message = message_frame.add_messages();
				message->set_robot_id(id);

				auto config = message->mutable_control_config();
				config->set_motor_kp(15.0f);
				config->set_motor_ki(1.0f);
				config->set_motor_kd(-20.0f);
				config->set_motor_i_limit(1000.0f);

				config->set_gyro_kp(5.0f);
				config->set_gyro_ki(0.0f);
				config->set_gyro_kd(0.5f);
				config->set_gyro_i_limit(0.0f);

				config->set_max_w_acc(1.0f);
				config->set_max_w_dec(1.0f);

			}
		}

		while (true)
		{
			uint8_t payload[7 * (MAX_PAYLOAD_SIZE + 1)];

			const size_t size = proto_msg_frame_to_byte_array(message_frame, payload);

			memset(payload + size, 0, MAX_PAYLOAD_SIZE + 1);
			payload[size] = 25;
			payload[size + 1] = 110;
			payload[size + 2] = 80;

			bool res = udp.send(payload, size + MAX_PAYLOAD_SIZE + 1, dest_address);

			printf("sendto (%lu): %d\n", size, res);

			Sleep(16);
		}
	};

	auto rcv_func = [&]()
	{
		Net::UDP udp;
		if (!udp.open(60006, false, false, true))
		{
			fprintf(stderr, "Unable to open UDP network port: %d\n", 60006);
			fflush(stderr);
			return(false);
		}

		Net::Address multiaddr, interf;
		multiaddr.setHost("224.5.23.3", 60006);
		
		interf.setAny();

		if (!udp.addMulticast(multiaddr, interf)) {
			fprintf(stderr, "Unable to setup UDP multicast\n");
			fflush(stderr);
			return(false);
		}

		uint8_t *in_buffer = new uint8_t[65536];

		while (true)
		{
			Net::Address src;
			int r = 0;
			r = udp.recv(in_buffer, 65536, src);
			if (r > 0) {
				fflush(stdout);
				//decode packet:
				printf("received %lu bytes from %s\n", r, "");

				Immortals::Data::RobotMessage message;
				feedback_bytes_to_proto_feedback(in_buffer, r, &message);

				const auto &feedback = message.feedback();

				//feedback.PrintDebugString();

				printf("targets : (%7.2f, %7.2f, %7.2f, %7.2f)\n",
					feedback.motor_target().x(),
					feedback.motor_target().y(),
					feedback.motor_target().z(),
					feedback.motor_target().w());

				printf("motors : (%7.2f, %7.2f, %7.2f, %7.2f)\n",
					feedback.motor_velocity().x(),
					feedback.motor_velocity().y(),
					feedback.motor_velocity().z(),
					feedback.motor_velocity().w());

				/*printf("ir  (%d) : %d\n",
					message.robot_id(),
					feedback.booster_enabled());*/

				/*printf("omega (%d) : %7.2f\n",
					message.robot_id(),
					feedback.omega());*/
				
			}
		}
	};

	std::thread send_thread(send_func);
	std::thread rcv_thread(rcv_func);

	send_thread.join();
	rcv_thread.join();

	return 0;
}
