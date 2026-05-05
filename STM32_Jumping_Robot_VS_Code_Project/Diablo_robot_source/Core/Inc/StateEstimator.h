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

extern float yaw_esp32 ;
extern float pitch_esp32  ;
extern float roll_esp32   ;
extern float gx_esp32   ;
extern float gy_esp32  ;
extern float gz_esp32   ;

#define TWO_PI (2.0f * M_PI)

