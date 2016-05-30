#pragma once

struct RobotCommand
{
	// commands
	float      vx;
	float      vy;
	float      omega;
	float      target_orientation;
	bool       halt;

	float      servo;
	float      shoot;
	float      chip;
	float      dribbler;

	float      orientation;

	// debug
	float      beep;
};

struct RobotConfig
{
	float      kp;
	float      ki;
	float      kd;
	float      i_limit;

	float      gyro_offset;
	float      head_offset;

	float      shoot_a;
	float      shoot_b;
	float      shoot_c;

	float      chip_a;
	float      chip_b;
	float      chip_c;
};

struct RobotFeedback
{
	float   battery_voltage;
	float   capacitor_voltage;

	bool    ball_detected;

	float   omega;
	float   orientation;

	bool    encoder_fault[4];
	bool    motor_fault[4];
	float   motor_velocity[4];
	float   motor_target[4];

	bool    button_status[8];
	bool    booster_enable;
	bool    dribbler_connected;
};