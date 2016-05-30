#pragma once

#include "data_lite.h"
#include <cstring>
#include <cstdint>
#include "half.h"

#include "defines.h"

inline void write_bytes(uint8_t* const buffer, size_t& pos, const void* const data, size_t count)
{
	memcpy(buffer + pos, data, count);
	pos += count;
}

inline void write_uint8(uint8_t* const buffer, size_t& pos, const uint8_t& data)
{
	buffer[pos] = data;
	pos += 1;
}

inline void write_uint16(uint8_t* const buffer, size_t& pos, const uint16_t& data)
{
	buffer[pos]     = data & 0x00FF;
	buffer[pos + 1] = data >> 8;
	pos += 2;
}

inline void write_uint32(uint8_t* const buffer, size_t& pos, const uint32_t& data)
{
	buffer[pos]     = data & 0x000000FF;
	buffer[pos + 1] = (data & 0x0000FF00) >> 8;
	buffer[pos + 2] = (data & 0x00FF0000) >> 16;
	buffer[pos + 3] = data >> 24;
	pos += 4;
}

inline void write_float_h(uint8_t* const buffer, size_t& pos, const FLOAT_32& data)
{
	write_uint16(buffer, pos, half_from_float(data.u32));
}

inline void write_float(uint8_t* const buffer, size_t& pos, const FLOAT_32& data)
{
	write_uint32(buffer, pos, data.u32);
}

inline void write_v2f_h(uint8_t* const buffer, size_t& pos, const Vector2f& data)
{
	write_float_h(buffer, pos, data.x);
	write_float_h(buffer, pos, data.y);
}

inline void write_v2f(uint8_t* const buffer, size_t& pos, const Vector2f& data)
{
	write_float(buffer, pos, data.x);
	write_float(buffer, pos, data.y);
}

inline void write_v3f_h(uint8_t* const buffer, size_t& pos, const Vector3f& data)
{
	write_float_h(buffer, pos, data.x);
	write_float_h(buffer, pos, data.y);
	write_float_h(buffer, pos, data.z);
}

inline void write_v3f(uint8_t* const buffer, size_t& pos, const Vector3f& data)
{
	write_float(buffer, pos, data.x);
	write_float(buffer, pos, data.y);
	write_float(buffer, pos, data.z);
}

inline void write_v4f_h(uint8_t* const buffer, size_t& pos, const Vector4f& data)
{
	write_float_h(buffer, pos, data.x);
	write_float_h(buffer, pos, data.y);
	write_float_h(buffer, pos, data.z);
	write_float_h(buffer, pos, data.w);
}

inline void write_v4f(uint8_t* const buffer, size_t& pos, const Vector4f& data)
{
	write_float(buffer, pos, data.x);
	write_float(buffer, pos, data.y);
	write_float(buffer, pos, data.z);
	write_float(buffer, pos, data.w);
}

inline size_t write_robot_command_fixed(uint8_t* const buffer, const RobotCommand& data)
{
	memset(buffer, 0, PAYLOAD_SIZE);

	size_t size = 0;

	write_uint8(buffer, size, MESSAGE_HEADER(PROTO_VERSION_FIXED, TYPE_COMMAND));

	write_v2f_h(buffer, size, data.velocity);

	write_float_h(buffer, size, data.omega);
	write_float_h(buffer, size, data.target_orientation);

	write_float_h(buffer, size, data.orientation);

	write_float_h(buffer, size, data.direct);
	write_float_h(buffer, size, data.chip);
	write_float_h(buffer, size, data.dribbler);
	write_float_h(buffer, size, data.servo);

	write_float_h(buffer, size, data.beep);

	write_uint8(buffer, size,
		(data.halt << 7) |
		((uint8_t)data.feedback) & 0x7F);

	return size;
}

inline size_t write_robot_config_fixed(uint8_t* const buffer, const RobotConfig& data)
{
	memset(buffer, 0, PAYLOAD_SIZE);

	size_t size = 0;

	write_uint8(buffer, size, MESSAGE_HEADER(PROTO_VERSION_FIXED, TYPE_CONFIG));

	write_float_h(buffer, size, data.kp);
	write_float_h(buffer, size, data.ki);
	write_float_h(buffer, size, data.kd);
	write_float_h(buffer, size, data.i_limit);

	write_float_h(buffer, size, data.gyro_offset);
	write_float_h(buffer, size, data.head_offset);

	write_v3f_h(buffer, size, data.direct_coeffs);
	write_v3f_h(buffer, size, data.chip_coeffs);

	return size;
}

inline size_t write_robot_matrix_fixed(uint8_t* const buffer, const RobotMatrix& data)
{
	memset(buffer, 0, PAYLOAD_SIZE);

	size_t size = 0;

	write_uint8(buffer, size, MESSAGE_HEADER(PROTO_VERSION_FIXED, TYPE_MATRIX));

	write_v3f_h(buffer, size, data.matrix[0]);
	write_v3f_h(buffer, size, data.matrix[1]);
	write_v3f_h(buffer, size, data.matrix[2]);
	write_v3f_h(buffer, size, data.matrix[3]);

	return size;
}

inline size_t write_robot_feedback_fixed(uint8_t* const buffer, const RobotFeedback& data)
{
	memset(buffer, 0, PAYLOAD_SIZE);

	size_t size = 0;

	write_uint8(buffer, size, MESSAGE_HEADER(PROTO_VERSION_FIXED, TYPE_FEEDBACK));

	write_float_h(buffer, size, data.battery_voltage);
	write_float_h(buffer, size, data.capacitor_voltage);

	write_float_h(buffer, size, data.omega);
	write_float_h(buffer, size, data.orientation);

	write_v4f_h(buffer, size, data.motor_velocity);
	write_v4f_h(buffer, size, data.motor_target);

	write_uint32(buffer, size,
		(data.ball_detected           ) |
		(data.fault              << 1 ) |
		(data.encoder_fault[0]   << 2 ) |
		(data.encoder_fault[1]   << 3 ) |
		(data.encoder_fault[2]   << 4 ) |
		(data.encoder_fault[3]   << 5 ) |
		(data.motor_fault[0]     << 6 ) |
		(data.motor_fault[1]     << 7 ) |
		(data.motor_fault[2]     << 8 ) |
		(data.motor_fault[3]     << 9 ) |
		(data.button_status[0]   << 10) |
		(data.button_status[1]   << 11) |
		(data.button_status[2]   << 12) |
		(data.button_status[3]   << 13) |
		(data.button_status[4]   << 14) |
		(data.button_status[5]   << 15) |
		(data.button_status[6]   << 16) |
		(data.button_status[7]   << 17) |
		(data.booster_enabled    << 18) |
		(data.dribbler_connected << 19));

	return size;
}

inline size_t write_robot_feedback_custom_fixed(uint8_t* const buffer, const RobotFeedbackCustom& data)
{
	memset(buffer, 0, PAYLOAD_SIZE);

	size_t size = 0;

	write_uint8(buffer, size, MESSAGE_HEADER(PROTO_VERSION_FIXED, TYPE_FEEDBACK_CUSTOM));

	write_uint8(buffer, size, data.length);
	write_bytes(buffer, size, data.debug_dump, data.length);
	
	return size;
}
