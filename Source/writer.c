#include "writer.h"

#include <string.h>
#include "half.h"
#include "defines.h"

void write_bytes(uint8_t* const buffer, size_t* const pos, const void* const data, const size_t count)
{
	memcpy(buffer + (*pos), data, count);
	*pos += count;
}

void write_bits8(uint8_t* const buffer, size_t* const pos, const struct bits8_t* const data)
{
	const uint8_t bits_data =
		data->bit0      |
		data->bit1 << 1 |
		data->bit2 << 2 |
		data->bit3 << 3 |
		data->bit4 << 4 |
		data->bit5 << 5 |
		data->bit6 << 6 |
		data->bit7 << 7 ;

	write_uint8(buffer, pos, bits_data);
}

void write_bits16(uint8_t* const buffer, size_t* const pos, const struct bits16_t* const data)
{
	const uint16_t bits_data =
		data->bit0        |
		data->bit1  << 1  |
		data->bit2  << 2  |
		data->bit3  << 3  |
		data->bit4  << 4  |
		data->bit5  << 5  |
		data->bit6  << 6  |
		data->bit7  << 7  |
		data->bit8  << 8  |
		data->bit9  << 9  |
		data->bit10 << 10 |
		data->bit11 << 11 |
		data->bit12 << 12 |
		data->bit13 << 13 |
		data->bit14 << 14 |
		data->bit15 << 15 ;

	write_uint16(buffer, pos, bits_data);
}

void write_bits32(uint8_t* const buffer, size_t* const pos, const struct bits32_t* const data)
{
	const uint32_t bits_data =
		data->bit0        |
		data->bit1  << 1  |
		data->bit2  << 2  |
		data->bit3  << 3  |
		data->bit4  << 4  |
		data->bit5  << 5  |
		data->bit6  << 6  |
		data->bit7  << 7  |
		data->bit8  << 8  |
		data->bit9  << 9  |
		data->bit10 << 10 |
		data->bit11 << 11 |
		data->bit12 << 12 |
		data->bit13 << 13 |
		data->bit14 << 14 |
		data->bit15 << 15 |
		data->bit16 << 16 |
		data->bit17 << 17 |
		data->bit18 << 18 |
		data->bit19 << 19 |
		data->bit20 << 20 |
		data->bit21 << 21 |
		data->bit22 << 22 |
		data->bit23 << 23 |
		data->bit24 << 24 |
		data->bit25 << 25 |
		data->bit26 << 26 |
		data->bit27 << 27 |
		data->bit28 << 28 |
		data->bit29 << 29 |
		data->bit30 << 30 |
		data->bit31 << 31 ;

	write_uint32(buffer, pos, bits_data);
}

void write_uint8(uint8_t* const buffer, size_t* const pos, const uint8_t data)
{
	buffer[*pos] = data;
	*pos += 1;
}

void write_uint16(uint8_t* const buffer, size_t* const pos, const uint16_t data)
{
	buffer[*pos] = data & 0x00FF;
	buffer[*pos + 1] = data >> 8;
	*pos += 2;
}

void write_uint32(uint8_t* const buffer, size_t* const pos, const uint32_t data)
{
	buffer[*pos] = data & 0x000000FF;
	buffer[*pos + 1] = (data & 0x0000FF00) >> 8;
	buffer[*pos + 2] = (data & 0x00FF0000) >> 16;
	buffer[*pos + 3] = data >> 24;
	*pos += 4;
}

void write_float_h(uint8_t* const buffer, size_t* const pos, const union float_32_u_t data)
{
	write_uint16(buffer, pos, half_from_float(data.u32));
}

void write_float(uint8_t* const buffer, size_t* const pos, const union float_32_u_t data)
{
	write_uint32(buffer, pos, data.u32);
}

void write_v2f_h(uint8_t* const buffer, size_t* const pos, const struct vector2f_t* const data)
{
	write_float_h(buffer, pos, data->x);
	write_float_h(buffer, pos, data->y);
}

void write_v2f(uint8_t* const buffer, size_t* const pos, const struct vector2f_t* const data)
{
	write_float(buffer, pos, data->x);
	write_float(buffer, pos, data->y);
}

void write_v3f_h(uint8_t* const buffer, size_t* const pos, const struct vector3f_t* const data)
{
	write_float_h(buffer, pos, data->x);
	write_float_h(buffer, pos, data->y);
	write_float_h(buffer, pos, data->z);
}

void write_v3f(uint8_t* const buffer, size_t* const pos, const struct vector3f_t* const data)
{
	write_float(buffer, pos, data->x);
	write_float(buffer, pos, data->y);
	write_float(buffer, pos, data->z);
}

void write_v4f_h(uint8_t* const buffer, size_t* const pos, const struct vector4f_t* const data)
{
	write_float_h(buffer, pos, data->x);
	write_float_h(buffer, pos, data->y);
	write_float_h(buffer, pos, data->z);
	write_float_h(buffer, pos, data->w);
}

void write_v4f(uint8_t* const buffer, size_t* const pos, const struct vector4f_t* const data)
{
	write_float(buffer, pos, data->x);
	write_float(buffer, pos, data->y);
	write_float(buffer, pos, data->z);
	write_float(buffer, pos, data->w);
}

size_t write_robot_command_fixed(uint8_t* const buffer, const struct robot_command_msg_t* const data)
{
	memset(buffer, 0, PAYLOAD_SIZE);

	size_t size = 0;

	write_uint8(buffer, &size, MESSAGE_HEADER(PROTO_VERSION_FIXED, TYPE_COMMAND));

	write_v2f_h(buffer, &size, &data->velocity);

	write_float_h(buffer, &size, data->omega);
	write_float_h(buffer, &size, data->target_orientation);

	write_float_h(buffer, &size, data->orientation);

	write_float_h(buffer, &size, data->shoot_power);
	write_float_h(buffer, &size, data->dribbler);
	write_float_h(buffer, &size, data->servo);

	write_uint8(buffer, &size, data->beep);

	const uint8_t packed_data = 
		((uint8_t)data->shoot_type) |
		((uint8_t)data->feedback) << 1 |
		(data->halt << 3) | 
		(data->has_orientation << 4);;
	write_uint8(buffer, &size, packed_data);

	return size;
}

size_t write_robot_config_fixed(uint8_t* const buffer, const struct robot_config_msg_t* const data)
{
	memset(buffer, 0, PAYLOAD_SIZE);

	size_t size = 0;

	write_uint8(buffer, &size, MESSAGE_HEADER(PROTO_VERSION_FIXED, TYPE_CONFIG));

	write_float_h(buffer, &size, data->kp);
	write_float_h(buffer, &size, data->ki);
	write_float_h(buffer, &size, data->kd);
	write_float_h(buffer, &size, data->i_limit);

	write_float_h(buffer, &size, data->head_offset);

	write_v3f_h(buffer, &size, &data->direct_coeffs);
	write_v3f_h(buffer, &size, &data->chip_coeffs);

	return size;
}

size_t write_robot_matrix_fixed(uint8_t* const buffer, const struct robot_matrix_msg_t* const data)
{
	memset(buffer, 0, PAYLOAD_SIZE);

	size_t size = 0;

	write_uint8(buffer, &size, MESSAGE_HEADER(PROTO_VERSION_FIXED, TYPE_MATRIX));

	write_v3f_h(buffer, &size, &data->matrix[0]);
	write_v3f_h(buffer, &size, &data->matrix[1]);
	write_v3f_h(buffer, &size, &data->matrix[2]);
	write_v3f_h(buffer, &size, &data->matrix[3]);

	return size;
}

size_t write_robot_feedback_fixed(uint8_t* const buffer, const struct robot_feedback_msg_t* const data)
{
	memset(buffer, 0, PAYLOAD_SIZE);

	size_t size = 0;

	write_uint8(buffer, &size, MESSAGE_HEADER(PROTO_VERSION_FIXED, TYPE_FEEDBACK));

	write_float_h(buffer, &size, data->battery_voltage);
	write_float_h(buffer, &size, data->capacitor_voltage);

	write_float_h(buffer, &size, data->omega);
	write_float_h(buffer, &size, data->orientation);

	write_v4f_h(buffer, &size, &data->motor_velocity);
	write_v4f_h(buffer, &size, &data->motor_target);

	write_bits8(buffer, &size, &data->motor_fault);
	write_bits8(buffer, &size, &data->button_status);

	const uint8_t packed_data =
		data->fault                   |
		data->ball_detected      << 1 |
		data->booster_enabled    << 2 |
		data->dribbler_connected << 3 ;

	write_uint8(buffer, &size, packed_data);

	return size;
}

size_t write_robot_feedback_custom_fixed(uint8_t* const buffer, const struct robot_feedback_custom_t* const data)
{
	memset(buffer, 0, PAYLOAD_SIZE);

	size_t size = 0;

	write_uint8(buffer, &size, MESSAGE_HEADER(PROTO_VERSION_FIXED, TYPE_FEEDBACK_CUSTOM));

	write_uint8(buffer, &size, data->length);
	write_bytes(buffer, &size, data->debug_dump, data->length);

	return size;
}
