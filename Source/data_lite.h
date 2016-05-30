#pragma once

#include "half.h"

struct Vector2f
{
	FLOAT_32 x;
	FLOAT_32 y;
};

struct Vector3f
{
	FLOAT_32 x;
	FLOAT_32 y;
	FLOAT_32 z;
};

struct Vector4f
{
	FLOAT_32 x;
	FLOAT_32 y;
	FLOAT_32 z;
	FLOAT_32 w;
};

enum TeamColor
{
	Blue = 0,
	Yellow = 1
};

struct RobotCommand
{
	// commands
	Vector2f   velocity;
	bool       halt;
	FLOAT_32      omega;
	FLOAT_32      target_orientation;

	FLOAT_32      orientation;

	FLOAT_32      direct;
	FLOAT_32      chip;
	FLOAT_32      dribbler;
	FLOAT_32      servo;

	// debug
	FLOAT_32      beep;

	enum FeedbackRequestType
	{
		None = 0,
		Debug = 1,
		Info = 2,
		Fatal = 3,
		Custom = 4
	};

	FeedbackRequestType feedback;
};

struct RobotConfig
{
	FLOAT_32      kp;
	FLOAT_32      ki;
	FLOAT_32      kd;
	FLOAT_32      i_limit;

	FLOAT_32      gyro_offset;
	FLOAT_32      head_offset;

	Vector3f   direct_coeffs;
	Vector3f   chip_coeffs;
};

struct RobotMatrix
{
	Vector3f matrix[4];
};

struct RobotFeedback
{
	FLOAT_32     battery_voltage;
	FLOAT_32     capacitor_voltage;

	bool      ball_detected;

	FLOAT_32     omega;
	FLOAT_32     orientation;

	bool      fault;
	bool      encoder_fault[4];
	bool      motor_fault[4];
	Vector4f  motor_velocity;
	Vector4f  motor_target;

	bool      button_status[8];
	bool      booster_enabled;
	bool      dribbler_connected;
};

struct RobotFeedbackCustom
{
	size_t    length;
	void*     debug_dump;
};
