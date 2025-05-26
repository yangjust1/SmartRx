/*
 * servo_control.c
 *
 *  Created on: Mar 24, 2023
 *      Author: xrico
 */
#include "stdio.h"
#include "math.h"
#include "stm32l4xx_hal.h"

// ####################################################################

#define KP 1 // Proportional Gain
#define KI 0.0075 // Integral Gain
#define KD 0.05 // Derivative Gain
#define KF 1510.0

#define OUTPUT_MAX 200

// ####################################################################

float clamp(float, float, float);
float getAngle();

void rotate360(int, int);
void calibrate(float, int);

// ####################################################################

extern float angle1, angle2, angle3, angle4; // CW motion ++, CCW motion -- (0 <= angle <= 359.99)
extern float rot1;
extern float prev1;
extern float diff1;
extern float num1;


extern void set_servo_speed(float, int);

// ####################################################################

static float dt = 0.001;
static float prev_position = 0.0; // set the initial position to zero
static float total_rotation = 0.0; // set the total rotation to zero

// ####################################################################

void calibrate(float target, int motor) {
	float error = 0;
	float prev_error = 0;
	float integral_error = 0;
	float derivative_error = 0;
	float desired_output = 0;

	int stable_loop = 0;
	int timeout = 0;

	while (1) {
		// Time Out
		++timeout;
		if (timeout > 750) {
			printf("Timed Out\n");
			break;
		}

		// Proportional
		error = target - angle1;

		// Integral
		if((desired_output) < OUTPUT_MAX && (desired_output) > -OUTPUT_MAX)
			integral_error += error * dt;

		// Derivative
		derivative_error = (error - prev_error) / dt;

		// PID Control
		desired_output = -(KP * error + KI * integral_error + KD * derivative_error);
		desired_output = clamp(desired_output, -200.00, 200.00);
		set_servo_speed(desired_output, motor);

		prev_error = error;

		if(error <= 1.0 && error >= -1.0) {
			set_servo_speed(0, motor);
			++stable_loop;

			if(stable_loop > 100)
				break;
			else {
				HAL_Delay(dt * 1000);
				continue;
			}
		}

		stable_loop = 0;

		HAL_Delay(dt * 1000);
	}
}

// ####################################################################


// ####################################################################

void rotate360(int direction, int motor) {
	float target = rot1 + direction * 360;
	float error = 0;
	float prev_error = 0;
	float integral_error = 0;
	float derivative_error = 0;
	float desired_output = 0;

	int stable_loop = 0;
	int timeout = 0;

	// printf("In Loop\n");

	while (1) {
		// Time Out
		++timeout;
		if (timeout > 1750) {
			printf("Timed Out\n");
			break;
		}

		// Proportional
		error = target - rot1;

		// Integral
		if((desired_output) < OUTPUT_MAX && (desired_output) > -OUTPUT_MAX)
			integral_error += error * dt;

		// Derivative
		derivative_error = (error - prev_error) / dt;

		// PID Control
		desired_output = -(KP * error + KI * integral_error + KD * derivative_error);
		desired_output = clamp(desired_output, -200.00, 200.00);
		set_servo_speed(desired_output, motor);

		prev_error = error;

		if(error <= 1.0 && error >= -1.0) {
			set_servo_speed(0, motor);
			++stable_loop;

			if(stable_loop > 100)
				break;
			else {
				HAL_Delay(dt * 1000);
				continue;
			}
		}

		stable_loop = 0;

		HAL_Delay(dt * 1000);
	}
}

// ####################################################################

float clamp(float d, float min, float max) {
  const float t = d < min ? min : d;
  return t > max ? max : t;
}

// #######################################################################

void dispense(uint8_t compartment, uint8_t quantity) {
	int motor = 0;
	motor &= quantity;

	if (quantity == 1) {
		rotate360(1, motor);
	} else {
		for (uint8_t i = 0; i < quantity/2; i++) {
			rotate360(1, motor);
			rotate360(-1, motor);
		}

		if (quantity % 2) {
			rotate360(1, motor);
		}
	}
}

