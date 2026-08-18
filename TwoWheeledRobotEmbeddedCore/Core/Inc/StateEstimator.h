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

// Body orientation derived from the BNO08x (bno08x.h), computed once per main-loop
// tick by StateEstimator_UpdateFromBNO(). pitch = fore/aft tilt, the balance axis
// consumed by the cascaded controller/fall detection; roll = lateral tilt; yaw = from
// the game rotation vector (no magnetometer, so it drifts slowly over long runs).
// Axis mapping (pitch = atan2(g_y,-g_z), roll = atan2(g_x,-g_z) on body-frame gravity,
// per STM32_DEPLOYMENT.md) assumes a specific BNO mounting orientation on the chassis
// that has not been bench-verified yet - confirm by tilting the robot and watching
// these move the expected direction before trusting balance behavior.
extern volatile float body_pitch_rad;
extern volatile float body_pitch_rate_rad_s;
extern volatile float body_roll_rad;
extern volatile float body_roll_rate_rad_s;
extern volatile float body_yaw_rad;
// Raw sensor/body gyro Z retained for diagnostics.
extern volatile float body_yaw_rate_rad_s;
// World-frame angular-velocity Z consumed by NN observation 6.
extern volatile float body_yaw_rate_world_rad_s;
// Independent fused-yaw finite-difference diagnostic.
extern volatile float body_yaw_rate_quat_rad_s;

void StateEstimator_UpdateFromBNO(void);

#define TWO_PI (2.0f * M_PI)

