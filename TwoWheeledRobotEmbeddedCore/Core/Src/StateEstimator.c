/*
 * StateEstimator.c
 *
 *  Created on: Oct 22, 2025
 *      Author: jon
 */

#include "StateEstimator.h"
#include "stdint.h"
#include "stdbool.h"
#include <math.h>
#include "bno08x.h"

volatile float estimated_theta_rad;
volatile float estimated_theta_dot_rad_s;
volatile float estimated_phi_dot_rad_s; // Assuming this is also global

volatile float body_pitch_rad;
volatile float body_pitch_rate_rad_s;
volatile float body_roll_rad;
volatile float body_roll_rate_rad_s;
volatile float body_yaw_rad;
volatile float body_yaw_rate_rad_s;

// Call once per main-loop tick, after BNO08x_Service(). No-ops until the BNO08x has
// produced its first gravity/gyro/rotation reports (bno_data_valid), so callers see
// zeros rather than garbage before the sensor comes up.
void StateEstimator_UpdateFromBNO(void)
{
    if (!bno_data_valid)
        return;

    body_pitch_rad       = atan2f(bno_gravity_y, -bno_gravity_z);
    body_pitch_rate_rad_s = bno_gx;
    body_roll_rad         = atan2f(bno_gravity_x, -bno_gravity_z);
    body_roll_rate_rad_s   = bno_gy;
    body_yaw_rad           = bno_yaw_rad;
    body_yaw_rate_rad_s     = bno_gz;
}


