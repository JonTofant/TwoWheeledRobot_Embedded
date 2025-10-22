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

void calculate_cascaded_motor_currents(float x_target_left, float x_target_right,
                                       float* current_motor1_out,
                                       float* current_motor2_out,
                                       float* total_force_out);

#endif /* INC_CONTROLER_H_ */
