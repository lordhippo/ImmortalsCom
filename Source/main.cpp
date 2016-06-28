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

		int robot_ids[6] = { 0, 1, 2, 3, 4, 5 };

		Immortals::Data::RobotMessageFrame message_frame;
		for (auto id : robot_ids)
		{
			auto message = message_frame.add_messages();
			message->set_robot_id(id);

			auto command = message->mutable_command();
			
			/*command->mutable_velocity()->set_x(0.0f);
			command->mutable_velocity()->set_y(40.0f);*/
			
			command->set_omega(200.0f);
			command->set_target_orientation(0.0f);
			/*command->set_orientation(0.0f);*/
			
			command->set_chip(80.0f);
			
			command->set_dribbler(0.0f);
			command->set_servo(0.0f);
			command->set_beep(0.0f);
			
			command->set_feedback(Immortals::Data::RobotCommand_FeedbackRequestType_Debug);
		}

		while (true)
		{
			uint8_t payload[7 * (MAX_PAYLOAD_SIZE + 1)];

			const size_t size = proto_msg_frame_to_byte_array(message_frame, payload);

			bool res = udp.send(payload, size, dest_address);

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

				Immortals::Data::RobotFeedback feedback;
				feedback_bytes_to_proto_feedback(in_buffer, r, &feedback);

				//feedback.PrintDebugString();

				/*printf("motors : (%7.2f, %7.2f, %7.2f, %7.2f)\n",
					feedback.motor_velocity().x(),
					feedback.motor_velocity().y(),
					feedback.motor_velocity().z(),
					feedback.motor_velocity().w());*/

				/*printf("ir : %d\n",
					feedback.ball_detected());*/

				printf("omega : %7.2f\n",
					feedback.omega());
				
			}
		}
	};

	std::thread send_thread(send_func);
	std::thread rcv_thread(rcv_func);

	send_thread.join();
	rcv_thread.join();

	return 0;
}
