#include <stdio.h>
#include "half.h"

#include "data_lite.h"
#include "writer.h"
#include "reader.h"
#include "defines.h"

void test_command()
{
	struct RobotCommand data;
	data.velocity.x.f32 = 100.5f;
	data.velocity.y.f32 = -254.0f;
	data.halt = TRUE;
	data.omega.f32 = 200.0f;
	data.target_orientation.f32 = 135.0f;
	data.orientation.f32 = 90.0f;
	data.shoot_power.f32 = 100.0f;
	data.dribbler.f32 = 95.0f;
	data.servo.f32 = 70.0f;
	data.beep = 190;
	data.shoot_type = SHOOT_TYPE_DIRECT;
	data.feedback = FEEDBACK_TYPE_DEBUG;

	uint8_t buffer[PAYLOAD_SIZE];
	size_t length = write_robot_command_fixed(buffer, &data);

	struct RobotCommand parsed_data;
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
		printf("--%s : [ %d : %d ]\n", "feedback", data.feedback, parsed_data.feedback);
	}
}

void test_config()
{
	struct RobotConfig data;
	data.kp.f32 = 1.0f;
	data.ki.f32 = -2.0f;
	data.kd.f32 = 3.0f;
	data.i_limit.f32 = -4.0f;

	data.head_offset.f32 = -6.0f;

	data.direct_coeffs.x.f32 = 100.0f;
	data.direct_coeffs.y.f32 = 150.0f;
	data.direct_coeffs.z.f32 = 250.0f;
	data.chip_coeffs.x.f32 = 1000.0f;
	data.chip_coeffs.y.f32 = 1050.0f;
	data.chip_coeffs.z.f32 = 2500.0f;

	uint8_t buffer[PAYLOAD_SIZE];
	size_t length = write_robot_config_fixed(buffer, &data);

	struct RobotConfig parsed_data;
	uint8_t result = read_robot_config_fixed(buffer, length, &parsed_data);

	printf("-Robot config [%lu] : %s\n", length, result == PARSE_RESULT_SUCCESS ? "pass" : "fail");

	if (result == PARSE_RESULT_SUCCESS)
	{
		printf("--%s : [ %.2f : %.2f ]\n", "kp", data.kp.f32, parsed_data.kp.f32);
		printf("--%s : [ %.2f : %.2f ]\n", "ki", data.ki.f32, parsed_data.ki.f32);
		printf("--%s : [ %.2f : %.2f ]\n", "kd", data.kd.f32, parsed_data.kd.f32);
		printf("--%s : [ %.2f : %.2f ]\n", "i_limit", data.i_limit.f32, parsed_data.i_limit.f32);
		printf("--%s : [ %.2f : %.2f ]\n", "head_offset", data.head_offset.f32, parsed_data.head_offset.f32);

		printf("--%s : [ %.2f : %.2f ]\n", "direct_coeffs.x", data.direct_coeffs.x.f32, parsed_data.direct_coeffs.x.f32);
		printf("--%s : [ %.2f : %.2f ]\n", "direct_coeffs.y", data.direct_coeffs.y.f32, parsed_data.direct_coeffs.y.f32);
		printf("--%s : [ %.2f : %.2f ]\n", "direct_coeffs.z", data.direct_coeffs.z.f32, parsed_data.direct_coeffs.z.f32);

		printf("--%s : [ %.2f : %.2f ]\n", "chip_coeffs.x", data.chip_coeffs.x.f32, parsed_data.chip_coeffs.x.f32);
		printf("--%s : [ %.2f : %.2f ]\n", "chip_coeffs.y", data.chip_coeffs.y.f32, parsed_data.chip_coeffs.y.f32);
		printf("--%s : [ %.2f : %.2f ]\n", "chip_coeffs.z", data.chip_coeffs.z.f32, parsed_data.chip_coeffs.z.f32);
	}
}

void test_matrix()
{
	struct RobotMatrix data;
	
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

	uint8_t buffer[PAYLOAD_SIZE];
	size_t length = write_robot_matrix_fixed(buffer, &data);

	struct RobotMatrix parsed_data;
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
	struct RobotFeedback data;
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

	data.ball_detected = TRUE;
	data.fault = TRUE;

	data.motor_fault.bit0 = FALSE;
	data.motor_fault.bit1 = FALSE;
	data.motor_fault.bit2 = FALSE;
	data.motor_fault.bit3 = TRUE;

	data.motor_fault.bit4 = FALSE;
	data.motor_fault.bit5 = FALSE;
	data.motor_fault.bit6 = TRUE;
	data.motor_fault.bit7 = FALSE;

	data.button_status.bit0 = FALSE;
	data.button_status.bit1 = FALSE;
	data.button_status.bit2 = FALSE;
	data.button_status.bit3 = FALSE;
	data.button_status.bit4 = TRUE;
	data.button_status.bit5 = FALSE;
	data.button_status.bit6 = TRUE;
	data.button_status.bit7 = TRUE;

	data.booster_enabled = TRUE;
	data.dribbler_connected = FALSE;

	uint8_t buffer[PAYLOAD_SIZE];
	size_t length = write_robot_feedback_fixed(buffer, &data);

	struct RobotFeedback parsed_data;
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

int main()
{
	test_command();
	test_config();
	test_matrix();
	test_feedback();

	return 0;
}
