#pragma once

#include "data_lite.h"
#include <cstring>
#include <cstdint>
#include "half.h"

#include "defines.h"

inline void read_bytes(const uint8_t* const buffer, size_t& pos, void* const data, size_t count)
{
	memcpy(data, buffer + pos, count);
	pos += count;
}

inline void read_uint8(const uint8_t* const buffer, size_t& pos, uint8_t& data)
{
	data = buffer[pos];
	pos += 1;
}

inline void read_uint16(const uint8_t* const buffer, size_t& pos, uint16_t& data)
{
	data =
		(buffer[pos]) |
		(buffer[pos + 1] << 8);

	pos += 2;
}

inline void read_uint32(const uint8_t* const buffer, size_t& pos, uint32_t& data)
{
	data =
		(buffer[pos]) |
		(buffer[pos + 1] << 8) |
		(buffer[pos + 2] << 16) |
		(buffer[pos + 3] << 24);

	pos += 4;
}

inline void read_float_h(const uint8_t* const buffer, size_t& pos, FLOAT_32& data)
{
	uint16_t h_data;
	read_uint16(buffer, pos, h_data);
	data.u32 = half_to_float(h_data);
}

inline void read_float(const uint8_t* const buffer, size_t& pos, FLOAT_32& data)
{
	read_uint32(buffer, pos, data.u32);
}

inline void read_v2f(const uint8_t* const buffer, size_t& pos, Vector2f& data)
{
	read_float(buffer, pos, data.x);
	read_float(buffer, pos, data.y);
}

inline void read_v2f_h(const uint8_t* const buffer, size_t& pos, Vector2f& data)
{
	read_float_h(buffer, pos, data.x);
	read_float_h(buffer, pos, data.y);
}

inline void read_v3f(const uint8_t* const buffer, size_t& pos, Vector3f& data)
{
	read_float(buffer, pos, data.x);
	read_float(buffer, pos, data.y);
	read_float(buffer, pos, data.z);
}

inline void read_v3f_h(const uint8_t* const buffer, size_t& pos, Vector3f& data)
{
	read_float_h(buffer, pos, data.x);
	read_float_h(buffer, pos, data.y);
	read_float_h(buffer, pos, data.z);
}

inline void read_v4f(const uint8_t* const buffer, size_t& pos, Vector4f& data)
{
	read_float(buffer, pos, data.x);
	read_float(buffer, pos, data.y);
	read_float(buffer, pos, data.z);
	read_float(buffer, pos, data.w);
}

inline void read_v4f_h(const uint8_t* const buffer, size_t& pos, Vector4f& data)
{
	read_float_h(buffer, pos, data.x);
	read_float_h(buffer, pos, data.y);
	read_float_h(buffer, pos, data.z);
	read_float_h(buffer, pos, data.w);
}

inline bool read_robot_command_fixed(const uint8_t* const buffer, const size_t size, RobotCommand& data)
{
	size_t pos = 0;

	uint8_t header;
	read_uint8(buffer, pos, header);
	if (header != MESSAGE_HEADER(PROTO_VERSION_FIXED, TYPE_COMMAND))
		return false;

	read_float_h(buffer, pos, data.velocity.x);
	read_float_h(buffer, pos, data.velocity.y);

	read_float_h(buffer, pos, data.omega);
	read_float_h(buffer, pos, data.target_orientation);

	read_float_h(buffer, pos, data.orientation);

	read_float_h(buffer, pos, data.direct);
	read_float_h(buffer, pos, data.chip);
	read_float_h(buffer, pos, data.dribbler);
	read_float_h(buffer, pos, data.servo);

	read_float_h(buffer, pos, data.beep);

	uint8_t feedback_i;
	read_uint8(buffer, pos, feedback_i);
	data.feedback = (RobotCommand::FeedbackRequestType) (feedback_i & 0x7F);
	data.halt = feedback_i & 0x01;

	return  (pos == size);
}

inline bool read_robot_config_fixed(const uint8_t* const buffer, const size_t size, RobotConfig& data)
{
	size_t pos = 0;

	uint8_t header;
	read_uint8(buffer, pos, header);
	if (header != MESSAGE_HEADER(PROTO_VERSION_FIXED, TYPE_CONFIG))
		return false;

	read_float_h(buffer, pos, data.kp);
	read_float_h(buffer, pos, data.ki);
	read_float_h(buffer, pos, data.kd);
	read_float_h(buffer, pos, data.i_limit);

	read_float_h(buffer, pos, data.gyro_offset);
	read_float_h(buffer, pos, data.head_offset);

	read_v3f_h(buffer, pos, data.direct_coeffs);
	read_v3f_h(buffer, pos, data.chip_coeffs);

	return  (pos == size);
}

inline bool read_robot_matrix_fixed(const uint8_t* const buffer, const size_t size, RobotMatrix& data)
{
	size_t pos = 0;

	uint8_t header;
	read_uint8(buffer, pos, header);
	if (header != MESSAGE_HEADER(PROTO_VERSION_FIXED, TYPE_MATRIX))
		return false;

	read_v3f_h(buffer, pos, data.matrix[0]);
	read_v3f_h(buffer, pos, data.matrix[1]);
	read_v3f_h(buffer, pos, data.matrix[2]);
	read_v3f_h(buffer, pos, data.matrix[3]);

	return  (pos == size);
}

inline bool read_robot_feedback_fixed(const uint8_t* const buffer, const size_t size, RobotFeedback& data)
{
	size_t pos = 0;

	uint8_t header;
	read_uint8(buffer, pos, header);
	if (header != MESSAGE_HEADER(PROTO_VERSION_FIXED, TYPE_FEEDBACK))
		return false;

	read_float_h(buffer, pos, data.battery_voltage);
	read_float_h(buffer, pos, data.capacitor_voltage);

	read_float_h(buffer, pos, data.omega);
	read_float_h(buffer, pos, data.orientation);

	read_v4f_h(buffer, pos, data.motor_velocity);
	read_v4f_h(buffer, pos, data.motor_target);

	uint32_t bitfield;
	read_uint32(buffer, pos, bitfield);

	data.ball_detected       = bitfield & 1;
	data.fault               = bitfield & (1 << 1 );
	data.encoder_fault[0]    = bitfield & (1 << 2 );
	data.encoder_fault[1]    = bitfield & (1 << 3 );
	data.encoder_fault[2]    = bitfield & (1 << 4 );
	data.encoder_fault[3]    = bitfield & (1 << 5 );
	data.motor_fault[0]      = bitfield & (1 << 6 );
	data.motor_fault[1]      = bitfield & (1 << 7 );
	data.motor_fault[2]      = bitfield & (1 << 8 );
	data.motor_fault[3]      = bitfield & (1 << 9 );
	data.button_status[0]    = bitfield & (1 << 10);
	data.button_status[1]    = bitfield & (1 << 11);
	data.button_status[2]    = bitfield & (1 << 12);
	data.button_status[3]    = bitfield & (1 << 13);
	data.button_status[4]    = bitfield & (1 << 14);
	data.button_status[5]    = bitfield & (1 << 15);
	data.button_status[6]    = bitfield & (1 << 16);
	data.button_status[7]    = bitfield & (1 << 17);
	data.booster_enabled     = bitfield & (1 << 18);
	data.dribbler_connected  = bitfield & (1 << 19);

	return  (pos == size);
}

inline bool read_robot_feedback_custom_fixed(const uint8_t* const buffer, const size_t size, RobotFeedbackCustom& data)
{
	size_t pos = 0;

	uint8_t header;
	read_uint8(buffer, pos, header);
	if (header != MESSAGE_HEADER(PROTO_VERSION_FIXED, TYPE_FEEDBACK_CUSTOM))
		return false;

	uint8_t data_length;
	read_uint8(buffer, pos, data_length);
	data.length = data_length;
	read_bytes(buffer, pos, data.debug_dump, data.length);

	return  (pos == size);
}
