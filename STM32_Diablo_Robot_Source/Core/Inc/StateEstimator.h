/*
 * StateEstimator.h
 *
 *  Created on: Oct 22, 2025
 *      Author: jon
 */

#include "DDSM115.h"

extern volatile float estimated_theta_rad;
extern volatile float estimated_theta_dot_rad_s;
extern volatile float estimated_phi_dot_rad_s; // Assuming this is also global

#define TWO_PI (2.0f * M_PI)

void update_base_state_estimates(const uint8_t* Buffer,
                                 volatile float* current_phi_rad,
                                 volatile float* current_phi_dot_rad_s,
                                 volatile float* current_x,
                                 volatile float* current_x_dot,
								 volatile float * current_x_ddot);
