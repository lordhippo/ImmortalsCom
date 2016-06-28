#include <stdio.h>
#include <thread>

#include "library/data_lite.h"
#include "library/writer.h"
#include "library/reader.h"
#include "library/defines.h"

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

		while (true)
		{
			struct robot_command_msg_t command_msg;
			command_msg.velocity.x.f32 = 0.0f;
			command_msg.velocity.y.f32 = 40.0f;
			command_msg.halt = true;
			command_msg.omega.f32 = 200.0f;
			command_msg.target_orientation.f32 = 0.0f;
			command_msg.orientation.f32 = 0.0f;
			command_msg.has_orientation = true;
			command_msg.shoot_power.f32 = 0.0f;
			command_msg.dribbler.f32 = 000.0f;
			command_msg.servo.f32 = 0.0f;
			command_msg.beep = 0;
			command_msg.shoot_type = SHOOT_TYPE_DIRECT;
			command_msg.feedback_request = FEEDBACK_TYPE_DEBUG;

			struct robot_wrapper_msg_t wrapper_msg;
			wrapper_msg.length = write_robot_command_fixed(wrapper_msg.data, &command_msg);

			uint8_t payload[7 * (MAX_PAYLOAD_SIZE + 1)];
			memset(payload, 0, 7 * (MAX_PAYLOAD_SIZE + 1));

			for (uint8_t i = 0; i < 6; i++)
			{
				payload[i*(MAX_PAYLOAD_SIZE + 1)] = i + 1;
				write_robot_wrapper_fixed(payload + (i*(MAX_PAYLOAD_SIZE + 1) + 1), &wrapper_msg);
			}

			payload[6 * (MAX_PAYLOAD_SIZE + 1)] = 25;
			payload[6 * (MAX_PAYLOAD_SIZE + 1) + 1] = 110;
			payload[6 * (MAX_PAYLOAD_SIZE + 1) + 2] = 80;

			bool res = udp.send(payload, 7 * (MAX_PAYLOAD_SIZE + 1), dest_address);

			//printf("sendto: %d\n", res);

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

		char *in_buffer = new char[65536];

		while (true)
		{
			Net::Address src;
			int r = 0;
			r = udp.recv(in_buffer, 65536, src);
			if (r > 0) {
				fflush(stdout);
				//decode packet:
				printf("received %lu bytes from %s\n", r, "");
			}
		}
	};

	std::thread send_thread(send_func);
	std::thread rcv_thread(rcv_func);

	send_thread.join();
	rcv_thread.join();

	return 0;
}
