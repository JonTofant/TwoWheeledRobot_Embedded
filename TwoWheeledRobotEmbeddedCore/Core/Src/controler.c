/*
 * LQR_Controller.c
 *
 *  Created on: Mar 18, 2025
 *      Author: tofan
 */


#include "controler.h"

// PITCH CONTROLLER (PID)
volatile float Kp_pitch = 0.0f;
volatile float Ki_pitch = 0.0f;
volatile float Kd_pitch = 0.0f;


float K_GAINS[2] = {100.0f,   10.0f};
float K_I_THETA = -18.0f; // Integral gain for theta


float Kp_pos_chasis = 1.2f;
float Kd_pos_chasis = -0.15f;
float Ki_pos_chasis = 0.6f;

float position_integral_L;
float position_integral_R;

// Position control gains for fall strategy
 float Kp_pos = 0.13f;
 float Kd_pos = -0.015f;
 float Ki_pos = 0.25f;

 float desired_v_left=0;
 float desired_v_right=0;

 //Target chasis position
 float target_x_chasis = 10.8/2;
 float target_y_chasis = -13.0;


 // Default values for DDSM115 motor current is 0 [A]
 float current_motor1_out= 0.0f;
 float current_motor2_out= 0.0f;
 float total_torque_out= 0.0f;
 float total_force_out = 0.0f;



void disable_controler(){
	Kp_pitch = 0.0f;
	Ki_pitch = 0.0f;
	Kd_pitch = 0.0f;

	K_GAINS[0] = 0.0f;
	K_GAINS[1] = 0.0f;
	K_I_THETA = 0.0f;

	Kp_pos_chasis = 0.0f;
	Kd_pos_chasis = 0.0f;
	Ki_pos_chasis = 0.0f;

	position_integral_L = 0.0f;
	position_integral_R = 0.0f;

	Kp_pos = 0.0f;
	Kd_pos = 0.0f;
	Ki_pos = 0.0f;

	current_motor1_out= 0.0f;
	current_motor2_out= 0.0f;
	total_torque_out= 0.0f;
	total_force_out = 0.0f;
}

void controler_defaults(){
	Kp_pitch = 0.0f;
	Ki_pitch = 0.0f;
	Kd_pitch = 0.0f;

	K_GAINS[0] = 100.0f;
	K_GAINS[1] = 10.0f;
	K_I_THETA = -18.0f;

	Kp_pos_chasis = 1.2f;
	Kd_pos_chasis = -0.15f;
	Ki_pos_chasis = 0.6f;

	position_integral_L = 0.0f;
	position_integral_R = 0.0f;

	Kp_pos = 0.13f;
	Kd_pos = -0.015f;
	Ki_pos = 0.25f;

	current_motor1_out= 0.0f;
	current_motor2_out= 0.0f;
	total_torque_out= 0.0f;
	total_force_out = 0.0f;
}
