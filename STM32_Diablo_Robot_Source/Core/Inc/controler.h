/*
 * controler.h
 *
 *  Created on: Mar 18, 2025
 *      Author: tofan
 */

#ifndef INC_CONTROLER_H_
#define INC_CONTROLER_H_

#include <math.h>
#include <stdint.h>
#include <stdio.h>

#include "cybergear.h"
#include "StateEstimator.h"
#include "telemetry.h"
#include "kinematics.h"


extern float sliding_surface_L;
extern float sliding_surface_R;


// SMC
extern float lambda;   // Convergence speed parameter
extern float K;       // SMC gain (must exceed max expected disturbance)
extern float phi;     // Boundary layer for chattering reduction



// Event based Variables
extern float event_threshold;        // tunable
extern float max_error_angle;        // experimental normalization constants
extern float max_error_angvel;
extern float max_error_vel;

extern volatile uint32_t event_trigger_count;
extern volatile uint32_t event_trigger_window_start;
extern float avg_trigger_rate_hz;
extern volatile float dt;

extern float d_e_angle_threshold;   // Threshold for angular error change (rad/s)
extern float d_e_angvel_threshold;  // Threshold for angular velocity error change (rad/s^2)
extern float d_e_vel_threshold;     // Threshold for linear velocity error change (m/s^2)

extern float e_angle;
extern float e_angvel;
extern float e_vel;

extern float e_norm;

// LQI gains for the cascade controler
extern float K_GAINS[2];
extern float K_I_THETA;

extern float Kp_pos_chasis;
extern float Kd_pos_chasis;
extern float Ki_pos_chasis;

extern float position_integral_L;
extern float position_integral_R;

// Position control gains for fall strategy
extern float Kp_pos;
extern float Kd_pos;
extern float Ki_pos;

extern float desired_v_left;
extern float desired_v_right;

//Target chasis position
extern float target_x_chasis;
extern float target_y_chasis;

// Current output for left ddsm115 motor
extern float current_motor1_out;
// Current output for right ddsm115 motor
extern float current_motor2_out;
// Total torque output
extern float total_torque_out;
// Calculated Force F output (optional log)
extern float total_force_out;

void update_pitch_leveling_controller(float current_pitch_rad, float dt);

void calculate_cascaded_motor_currents_smc(float x_target_left, float x_target_right,
                                           float sliding_surface_left, float sliding_surface_right,
                                           float* current_motor1_out,
                                           float* current_motor2_out,
                                           float* total_force_out);

#endif /* INC_CONTROLER_H_ */
