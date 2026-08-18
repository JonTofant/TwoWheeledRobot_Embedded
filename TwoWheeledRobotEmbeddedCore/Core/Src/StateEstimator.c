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
volatile float body_yaw_rate_world_rad_s;
volatile float body_yaw_rate_quat_rad_s;

static float wrap_delta_pi(float angle_rad)
{
    while (angle_rad > (float)M_PI)
        angle_rad -= 2.0f * (float)M_PI;
    while (angle_rad < -(float)M_PI)
        angle_rad += 2.0f * (float)M_PI;
    return angle_rad;
}

// Call once per main-loop tick, after BNO08x_Service(). No-ops until the BNO08x has
// produced its first gravity/gyro/rotation reports (bno_data_valid), so callers see
// zeros rather than garbage before the sensor comes up.
void StateEstimator_UpdateFromBNO(void)
{
    static uint32_t previous_rotation_count;
    static uint64_t previous_rotation_timestamp_us;
    static float previous_yaw_rad;

    if (!bno_data_valid)
        return;

    body_pitch_rad       = atan2f(bno_gravity_y, -bno_gravity_z);
    /* With Z up and pitch derived from gravity Y as above, increasing pitch
     * corresponds to negative right-hand rotation about sensor/body X. */
    body_pitch_rate_rad_s = -bno_gx;
    body_roll_rad         = atan2f(bno_gravity_x, -bno_gravity_z);
    body_roll_rate_rad_s   = bno_gy;
    body_yaw_rad           = bno_yaw_rad;
    body_yaw_rate_rad_s     = bno_gz;  // Raw body/sensor-frame diagnostic.

    /* Rotate the complete body-frame gyro vector into the quaternion's world
     * frame and retain its Z component. The NN policy consumes this value;
     * the fused-yaw derivative below remains an independent diagnostic. */
    float qw = bno_qw;
    float qx = bno_qx;
    float qy = bno_qy;
    float qz = bno_qz;
    float q_norm_sq = qw*qw + qx*qx + qy*qy + qz*qz;
    if (q_norm_sq > 1.0e-8f)
    {
        float inv_q_norm = 1.0f / sqrtf(q_norm_sq);
        qw *= inv_q_norm;
        qx *= inv_q_norm;
        qy *= inv_q_norm;
        qz *= inv_q_norm;

        float r20 = 2.0f * (qx*qz - qw*qy);
        float r21 = 2.0f * (qy*qz + qw*qx);
        float r22 = 1.0f - 2.0f * (qx*qx + qy*qy);
        body_yaw_rate_world_rad_s = r20*bno_gx + r21*bno_gy + r22*bno_gz;
    }
    else
    {
        body_yaw_rate_world_rad_s = bno_gz;
    }

    /* Independent reference: finite-difference the fused yaw only when a new
     * rotation-vector report arrives, using the SH-2 report timestamp. */
    uint32_t rotation_count = bno_rotation_update_count;
    uint64_t rotation_timestamp_us = bno_rotation_timestamp_us;
    if (rotation_count != previous_rotation_count)
    {
        if (previous_rotation_count != 0u &&
            rotation_timestamp_us > previous_rotation_timestamp_us)
        {
            float dt_s = (float)(rotation_timestamp_us - previous_rotation_timestamp_us) * 1.0e-6f;
            if (dt_s > 1.0e-4f && dt_s < 0.25f)
            {
                body_yaw_rate_quat_rad_s =
                    wrap_delta_pi(body_yaw_rad - previous_yaw_rad) / dt_s;
            }
        }
        previous_yaw_rad = body_yaw_rad;
        previous_rotation_timestamp_us = rotation_timestamp_us;
        previous_rotation_count = rotation_count;
    }
}


