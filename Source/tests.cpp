#include "tests.h"

#include <stdio.h>

#include "library/data_lite.h"
#include "library/reader.h"
#include "library/writer.h"
#include "library/defines.h"

void test_command()
{
	struct robot_command_msg_t data;
	data.velocity.x.f32 = 100.5f;
	data.velocity.y.f32 = -254.0f;
	data.halt = true;
	data.omega.f32 = 200.0f;
	data.target_orientation.f32 = 135.0f;
	data.orientation.f32 = 90.0f;
	data.shoot_power.f32 = 100.0f;
	data.dribbler.f32 = 95.0f;
	data.servo.f32 = 70.0f;
	data.beep = 190;
	data.shoot_type = SHOOT_TYPE_DIRECT;
	data.feedback_request = FEEDBACK_TYPE_DEBUG;

	uint8_t buffer[MAX_PAYLOAD_SIZE];
	size_t length = write_robot_command_fixed(buffer, &data);

	struct robot_command_msg_t parsed_data;
	uint8_t result = read_robot_command_fixed(buffer, length, &parsed_data);

	printf("-Robot command [%lu] : %s\n", length, result == PARSE_RESULT_SUCCESS ? "pass" : "fail");
	if (result == PARSE_RESULT_SUCCESS)
	{
		printf("--%s : [ %.2f : %.2f ]\n", "velocity.x", data.velocity.x.f32, parsed_data.velocity.x.f32);
		printf("--%s : [ %.2f : %.2f ]\n", "velocity.y", data.velocity.y.f32, parsed_data.velocity.y.f32);
		printf("--%s : [ %d : %d ]\n", "halt", data.halt, parsed_data.halt);
		printf("--%s : [ %.2f : %.2f ]\n", "omega", data.omega.f32, parsed_data.omega.f32);
		printf("--%s : [ %.2f : %.2f ]\n", "target_orientation", data.target_orientation.f32, parsed_data.target_orientation.f32);
		printf("--%s : [ %.2f : %.2f ]\n", "orientation", data.orientation.f32, parsed_data.orientation.f32);
		printf("--%s : [ %.2f : %.2f ]\n", "direct", data.shoot_power.f32, parsed_data.shoot_power.f32);
		printf("--%s : [ %.2f : %.2f ]\n", "dribbler", data.dribbler.f32, parsed_data.dribbler.f32);
		printf("--%s : [ %.2f : %.2f ]\n", "servo", data.servo.f32, parsed_data.servo.f32);
		printf("--%s : [ %hhu : %hhu ]\n", "beep", data.beep, parsed_data.beep);
		printf("--%s : [ %d : %d ]\n", "shoot_type", data.shoot_type, parsed_data.shoot_type);
		printf("--%s : [ %d : %d ]\n", "feedback", data.feedback_request, parsed_data.feedback_request);
	}
}

void test_config_control()
{
	struct robot_control_config_msg_t data;
	data.motor_kp.f32 = 1.0f;
	data.motor_ki.f32 = -2.0f;
	data.motor_kd.f32 = 3.0f;
	data.motor_i_limit.f32 = -4.0f;

	data.gyro_kp.f32 = 11.0f;
	data.gyro_ki.f32 = -22.0f;
	data.gyro_kd.f32 = 36.0f;
	data.gyro_i_limit.f32 = -14.012f;

	data.max_w_acc.f32 = 336.10f;
	data.max_w_dec.f32 = -154.03f;

	uint8_t buffer[MAX_PAYLOAD_SIZE];
	size_t length = write_robot_control_config_fixed(buffer, &data);

	struct robot_control_config_msg_t parsed_data;
	uint8_t result = read_robot_control_config_fixed(buffer, length, &parsed_data);

	printf("-Robot control config [%lu] : %s\n", length, result == PARSE_RESULT_SUCCESS ? "pass" : "fail");

	if (result == PARSE_RESULT_SUCCESS)
	{
		printf("--%s : [ %.2f : %.2f ]\n", "motor kp", data.motor_kp.f32, parsed_data.motor_kp.f32);
		printf("--%s : [ %.2f : %.2f ]\n", "motor ki", data.motor_ki.f32, parsed_data.motor_ki.f32);
		printf("--%s : [ %.2f : %.2f ]\n", "motor kd", data.motor_kd.f32, parsed_data.motor_kd.f32);
		printf("--%s : [ %.2f : %.2f ]\n", "motor i_limit", data.motor_i_limit.f32, parsed_data.motor_i_limit.f32);

		printf("--%s : [ %.2f : %.2f ]\n", "gyro kp", data.gyro_kp.f32, parsed_data.gyro_kp.f32);
		printf("--%s : [ %.2f : %.2f ]\n", "gyro ki", data.gyro_ki.f32, parsed_data.gyro_ki.f32);
		printf("--%s : [ %.2f : %.2f ]\n", "gyro kd", data.gyro_kd.f32, parsed_data.gyro_kd.f32);
		printf("--%s : [ %.2f : %.2f ]\n", "gyro i_limit", data.gyro_i_limit.f32, parsed_data.gyro_i_limit.f32);

		printf("--%s : [ %.2f : %.2f ]\n", "max_w_acc", data.max_w_acc.f32, parsed_data.max_w_acc.f32);
		printf("--%s : [ %.2f : %.2f ]\n", "max_w_dec", data.max_w_dec.f32, parsed_data.max_w_dec.f32);
	}
}

void test_config_shoot()
{
	struct robot_shoot_config_msg_t data;
	data.direct_coeffs.x.f32 = 100.0f;
	data.direct_coeffs.y.f32 = 150.0f;
	data.direct_coeffs.z.f32 = 250.0f;
	data.chip_coeffs.x.f32 = 1000.0f;
	data.chip_coeffs.y.f32 = 1050.0f;
	data.chip_coeffs.z.f32 = 2500.0f;

	uint8_t buffer[MAX_PAYLOAD_SIZE];
	size_t length = write_robot_shoot_config_fixed(buffer, &data);

	struct robot_shoot_config_msg_t parsed_data;
	uint8_t result = read_robot_shoot_config_fixed(buffer, length, &parsed_data);

	printf("-Robot shoot config [%lu] : %s\n", length, result == PARSE_RESULT_SUCCESS ? "pass" : "fail");

	if (result == PARSE_RESULT_SUCCESS)
	{
		printf("--%s : [ %.2f : %.2f ]\n", "direct_coeffs.x", data.direct_coeffs.x.f32, parsed_data.direct_coeffs.x.f32);
		printf("--%s : [ %.2f : %.2f ]\n", "direct_coeffs.y", data.direct_coeffs.y.f32, parsed_data.direct_coeffs.y.f32);
		printf("--%s : [ %.2f : %.2f ]\n", "direct_coeffs.z", data.direct_coeffs.z.f32, parsed_data.direct_coeffs.z.f32);

		printf("--%s : [ %.2f : %.2f ]\n", "chip_coeffs.x", data.chip_coeffs.x.f32, parsed_data.chip_coeffs.x.f32);
		printf("--%s : [ %.2f : %.2f ]\n", "chip_coeffs.y", data.chip_coeffs.y.f32, parsed_data.chip_coeffs.y.f32);
		printf("--%s : [ %.2f : %.2f ]\n", "chip_coeffs.z", data.chip_coeffs.z.f32, parsed_data.chip_coeffs.z.f32);
	}
}

void test_config_on_board()
{
	struct robot_on_board_config_t data;

	data.control_config.motor_kp.f32 = 1.0f;
	data.control_config.motor_ki.f32 = -2.0f;
	data.control_config.motor_kd.f32 = 3.0f;
	data.control_config.motor_i_limit.f32 = -4.0f;

	data.control_config.gyro_kp.f32 = 11.0f;
	data.control_config.gyro_ki.f32 = -22.0f;
	data.control_config.gyro_kd.f32 = 36.0f;
	data.control_config.gyro_i_limit.f32 = -14.012f;

	data.control_config.max_w_acc.f32 = 336.10f;
	data.control_config.max_w_dec.f32 = -154.03f;

	data.shoot_config.direct_coeffs.x.f32 = 100.0f;
	data.shoot_config.direct_coeffs.y.f32 = 150.0f;
	data.shoot_config.direct_coeffs.z.f32 = 250.0f;
	data.shoot_config.chip_coeffs.x.f32 = 1000.0f;
	data.shoot_config.chip_coeffs.y.f32 = 1050.0f;
	data.shoot_config.chip_coeffs.z.f32 = 2500.0f;

	data.gyro_offset.f32 = 0.512f;
	data.nrf_channel_rx = 110;
	data.nrf_channel_tx = 80;
	data.use_encoders = 0;

	uint8_t buffer[MAX_ON_BOARD_SIZE];
	size_t length = write_robot_on_board_config_fixed(buffer, &data);

	struct robot_on_board_config_t parsed_data;
	uint8_t result = read_robot_on_board_config_fixed(buffer, length, &parsed_data);

	printf("-Robot on-board config [%lu] : %s\n", length, result == PARSE_RESULT_SUCCESS ? "pass" : "fail");

	if (result == PARSE_RESULT_SUCCESS)
	{
		printf("--%s : [ %.2f : %.2f ]\n", "motor kp", data.control_config.motor_kp.f32, parsed_data.control_config.motor_kp.f32);
		printf("--%s : [ %.2f : %.2f ]\n", "motor ki", data.control_config.motor_ki.f32, parsed_data.control_config.motor_ki.f32);
		printf("--%s : [ %.2f : %.2f ]\n", "motor kd", data.control_config.motor_kd.f32, parsed_data.control_config.motor_kd.f32);
		printf("--%s : [ %.2f : %.2f ]\n", "motor i_limit", data.control_config.motor_i_limit.f32, parsed_data.control_config.motor_i_limit.f32);

		printf("--%s : [ %.2f : %.2f ]\n", "gyro kp", data.control_config.gyro_kp.f32, parsed_data.control_config.gyro_kp.f32);
		printf("--%s : [ %.2f : %.2f ]\n", "gyro ki", data.control_config.gyro_ki.f32, parsed_data.control_config.gyro_ki.f32);
		printf("--%s : [ %.2f : %.2f ]\n", "gyro kd", data.control_config.gyro_kd.f32, parsed_data.control_config.gyro_kd.f32);
		printf("--%s : [ %.2f : %.2f ]\n", "gyro i_limit", data.control_config.gyro_i_limit.f32, parsed_data.control_config.gyro_i_limit.f32);

		printf("--%s : [ %.2f : %.2f ]\n", "max_w_acc", data.control_config.max_w_acc.f32, parsed_data.control_config.max_w_acc.f32);
		printf("--%s : [ %.2f : %.2f ]\n", "max_w_dec", data.control_config.max_w_dec.f32, parsed_data.control_config.max_w_dec.f32);

		printf("--%s : [ %.2f : %.2f ]\n", "direct_coeffs.x", data.shoot_config.direct_coeffs.x.f32, parsed_data.shoot_config.direct_coeffs.x.f32);
		printf("--%s : [ %.2f : %.2f ]\n", "direct_coeffs.y", data.shoot_config.direct_coeffs.y.f32, parsed_data.shoot_config.direct_coeffs.y.f32);
		printf("--%s : [ %.2f : %.2f ]\n", "direct_coeffs.z", data.shoot_config.direct_coeffs.z.f32, parsed_data.shoot_config.direct_coeffs.z.f32);

		printf("--%s : [ %.2f : %.2f ]\n", "chip_coeffs.x", data.shoot_config.chip_coeffs.x.f32, parsed_data.shoot_config.chip_coeffs.x.f32);
		printf("--%s : [ %.2f : %.2f ]\n", "chip_coeffs.y", data.shoot_config.chip_coeffs.y.f32, parsed_data.shoot_config.chip_coeffs.y.f32);
		printf("--%s : [ %.2f : %.2f ]\n", "chip_coeffs.z", data.shoot_config.chip_coeffs.z.f32, parsed_data.shoot_config.chip_coeffs.z.f32);

		printf("--%s : [ %.2f : %.2f ]\n", "gyro_offset", data.gyro_offset.f32, parsed_data.gyro_offset.f32);
		printf("--%s : [ %u : %u ]\n", "nrf_channel_rx", data.nrf_channel_rx, parsed_data.nrf_channel_rx);
		printf("--%s : [ %u : %u ]\n", "nrf_channel_tx", data.nrf_channel_tx, parsed_data.nrf_channel_tx);
		printf("--%s : [ %u : %u ]\n", "use_encoders", data.use_encoders, parsed_data.use_encoders);
	}
}

void test_matrix()
{
	struct robot_matrix_msg_t data;

	data.matrix[0].x.f32 = 1.0f;
	data.matrix[0].y.f32 = -2.0f;
	data.matrix[0].z.f32 = 3.0f;

	data.matrix[1].x.f32 = -4.0f;
	data.matrix[1].y.f32 = 5.0f;
	data.matrix[1].z.f32 = -6.0f;

	data.matrix[2].x.f32 = 100.0f;
	data.matrix[2].y.f32 = 150.0f;
	data.matrix[2].z.f32 = 250.0f;

	data.matrix[3].x.f32 = 1000.0f;
	data.matrix[3].y.f32 = 1050.0f;
	data.matrix[3].z.f32 = 2500.0f;

	uint8_t buffer[MAX_PAYLOAD_SIZE];
	size_t length = write_robot_matrix_fixed(buffer, &data);

	struct robot_matrix_msg_t parsed_data;
	uint8_t result = read_robot_matrix_fixed(buffer, length, &parsed_data);

	printf("-Robot matrix [%lu] : %s\n", length, result == PARSE_RESULT_SUCCESS ? "pass" : "fail");

	if (result == PARSE_RESULT_SUCCESS)
	{
		printf("--%s : [ %.2f : %.2f ]\n", "matrix[0].x", data.matrix[0].x.f32, parsed_data.matrix[0].x.f32);
		printf("--%s : [ %.2f : %.2f ]\n", "matrix[0].y", data.matrix[0].y.f32, parsed_data.matrix[0].y.f32);
		printf("--%s : [ %.2f : %.2f ]\n", "matrix[0].z", data.matrix[0].z.f32, parsed_data.matrix[0].z.f32);

		printf("--%s : [ %.2f : %.2f ]\n", "matrix[1].x", data.matrix[1].x.f32, parsed_data.matrix[1].x.f32);
		printf("--%s : [ %.2f : %.2f ]\n", "matrix[1].y", data.matrix[1].y.f32, parsed_data.matrix[1].y.f32);
		printf("--%s : [ %.2f : %.2f ]\n", "matrix[1].z", data.matrix[1].z.f32, parsed_data.matrix[1].z.f32);

		printf("--%s : [ %.2f : %.2f ]\n", "matrix[2].x", data.matrix[2].x.f32, parsed_data.matrix[2].x.f32);
		printf("--%s : [ %.2f : %.2f ]\n", "matrix[2].y", data.matrix[2].y.f32, parsed_data.matrix[2].y.f32);
		printf("--%s : [ %.2f : %.2f ]\n", "matrix[2].z", data.matrix[2].z.f32, parsed_data.matrix[2].z.f32);

		printf("--%s : [ %.2f : %.2f ]\n", "matrix[3].x", data.matrix[3].x.f32, parsed_data.matrix[3].x.f32);
		printf("--%s : [ %.2f : %.2f ]\n", "matrix[3].y", data.matrix[3].y.f32, parsed_data.matrix[3].y.f32);
		printf("--%s : [ %.2f : %.2f ]\n", "matrix[3].z", data.matrix[3].z.f32, parsed_data.matrix[3].z.f32);
	}
}

void test_feedback()
{
	struct robot_feedback_msg_t data;
	data.battery_voltage.f32 = 1.0f;
	data.capacitor_voltage.f32 = -2.0f;
	data.omega.f32 = 3.0f;
	data.orientation.f32 = -4.0f;

	data.motor_velocity.x.f32 = 5.0f;
	data.motor_velocity.y.f32 = -6.0f;
	data.motor_velocity.z.f32 = 100.0f;
	data.motor_velocity.w.f32 = 150.0f;

	data.motor_target.x.f32 = 250.0f;
	data.motor_target.y.f32 = 1000.0f;
	data.motor_target.z.f32 = 1050.0f;
	data.motor_target.w.f32 = 2500.0f;

	data.ball_detected = true;
	data.fault = true;

	data.motor_fault.bit0 = false;
	data.motor_fault.bit1 = false;
	data.motor_fault.bit2 = false;
	data.motor_fault.bit3 = true;

	data.motor_fault.bit4 = false;
	data.motor_fault.bit5 = false;
	data.motor_fault.bit6 = true;
	data.motor_fault.bit7 = false;

	data.button_status.bit0 = false;
	data.button_status.bit1 = false;
	data.button_status.bit2 = false;
	data.button_status.bit3 = false;
	data.button_status.bit4 = true;
	data.button_status.bit5 = false;
	data.button_status.bit6 = true;
	data.button_status.bit7 = true;

	data.booster_enabled = true;
	data.dribbler_connected = false;

	uint8_t buffer[MAX_PAYLOAD_SIZE];
	size_t length = write_robot_feedback_fixed(buffer, &data, FEEDBACK_TYPE_DEBUG);

	struct robot_feedback_msg_t parsed_data;
	uint8_t result = read_robot_feedback_fixed(buffer, length, &parsed_data);

	printf("-Robot feedback [%lu] : %s\n", length, result == PARSE_RESULT_SUCCESS ? "pass" : "fail");

	if (result == PARSE_RESULT_SUCCESS)
	{
		printf("--%s : [ %.2f : %.2f ]\n", "battery_voltage", data.battery_voltage.f32, parsed_data.battery_voltage.f32);
		printf("--%s : [ %.2f : %.2f ]\n", "capacitor_voltage", data.capacitor_voltage.f32, parsed_data.capacitor_voltage.f32);
		printf("--%s : [ %.2f : %.2f ]\n", "omega", data.omega.f32, parsed_data.omega.f32);
		printf("--%s : [ %.2f : %.2f ]\n", "orientation", data.orientation.f32, parsed_data.orientation.f32);

		printf("--%s : [ %.2f : %.2f ]\n", "motor_velocity.x", data.motor_velocity.x.f32, parsed_data.motor_velocity.x.f32);
		printf("--%s : [ %.2f : %.2f ]\n", "motor_velocity.y", data.motor_velocity.y.f32, parsed_data.motor_velocity.y.f32);
		printf("--%s : [ %.2f : %.2f ]\n", "motor_velocity.z", data.motor_velocity.z.f32, parsed_data.motor_velocity.z.f32);
		printf("--%s : [ %.2f : %.2f ]\n", "motor_velocity.w", data.motor_velocity.w.f32, parsed_data.motor_velocity.w.f32);

		printf("--%s : [ %.2f : %.2f ]\n", "motor_target.x", data.motor_target.x.f32, parsed_data.motor_target.x.f32);
		printf("--%s : [ %.2f : %.2f ]\n", "motor_target.y", data.motor_target.y.f32, parsed_data.motor_target.y.f32);
		printf("--%s : [ %.2f : %.2f ]\n", "motor_target.z", data.motor_target.z.f32, parsed_data.motor_target.z.f32);
		printf("--%s : [ %.2f : %.2f ]\n", "motor_target.w", data.motor_target.w.f32, parsed_data.motor_target.w.f32);

		printf("--%s : [ %d : %d ]\n", "orientation", data.ball_detected, parsed_data.ball_detected);
		printf("--%s : [ %d : %d ]\n", "fault", data.fault, parsed_data.fault);

		printf("--%s : [ %d : %d ]\n", "booster_enabled", data.booster_enabled, parsed_data.booster_enabled);
		printf("--%s : [ %d : %d ]\n", "dribbler_connected", data.dribbler_connected, parsed_data.dribbler_connected);
	}
}