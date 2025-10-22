/*
 * StateEstimator.c
 *
 *  Created on: Oct 22, 2025
 *      Author: jon
 */

#include "StateEstimator.h"
#include "stdint.h"
#include "stdbool.h"

volatile float estimated_theta_rad;
volatile float estimated_theta_dot_rad_s;
volatile float estimated_phi_dot_rad_s; // Assuming this is also global

float yaw_esp32 ;
float pitch_esp32  ;
float roll_esp32   ;
float gx_esp32   ;
float gy_esp32  ;
float gz_esp32   ;

void update_base_state_estimates(const uint8_t* Buffer,
                                 volatile float* current_phi_rad,
                                 volatile float* current_phi_dot_rad_s,
                                 volatile float* current_x,
                                 volatile float* current_x_dot,
								 volatile float * current_x_ddot)
{
    // Static variables for phi calculation persist state between calls FOR THIS MOTOR
    static int32_t phi_num_rotations = 0;
    static uint16_t phi_prev_raw_pos = 0;
    static bool phi_initialized = false;

    // 1. Check if the feedback is from the designated motor (e.g., 0x10)
    uint8_t motor_id = Buffer[0];
    if (motor_id != 0x10) { // <<< Use #define MOTOR_ID_FOR_PHI_DOT if defined
        return;
    }

    // --- Proceed only if data is from the correct motor ---

    // 2. Extract and calculate ANGULAR Velocity (Phi_dot)
    uint16_t raw_velocity = ((uint16_t)Buffer[4] << 8) | Buffer[5];
    int16_t velocityDDSM_RPM = (int16_t)raw_velocity;
    // Apply sign correction if needed based on wiring/definition
    *current_phi_dot_rad_s = -(float)velocityDDSM_RPM * RPM_TO_RAD_PER_SEC;

    // 3. Calculate LINEAR Velocity (x_dot)
    *current_x_dot = *current_phi_dot_rad_s * WHEEL_RADIUS_R;

    // 4. Extract current raw ANGULAR position
    uint16_t current_raw_pos = ((uint16_t)Buffer[6] << 8) | Buffer[7];

    // 5. Handle ANGULAR POSITION initialization
    if (!phi_initialized) {
        phi_prev_raw_pos = current_raw_pos;
        if (fabsf(*current_phi_rad) < 1e-6f) { // Use global phi_rad for initial check
             *current_phi_rad = (float)current_raw_pos * (TWO_PI / RAW_POS_MAX_COUNT) ;
        }
        phi_num_rotations = roundf(*current_phi_rad / TWO_PI); // Estimate initial rotations
        phi_initialized = true;
        // Ensure x is also initialized correctly on the first run
        *current_x = *current_phi_rad * WHEEL_RADIUS_R;
    } else {
        // 6. Calculate ANGULAR POSITION wrap-around and continuous angle
        int32_t delta_raw = (int32_t)current_raw_pos - (int32_t)phi_prev_raw_pos;
        if (delta_raw > RAW_POS_HALF_RANGE) { phi_num_rotations--; }
        else if (delta_raw < -RAW_POS_HALF_RANGE) { phi_num_rotations++; }

        float current_angle_base_rad = (float)current_raw_pos * (TWO_PI / RAW_POS_MAX_COUNT);
        *current_phi_rad = (current_angle_base_rad + (float)phi_num_rotations * TWO_PI)-12.355f;

        // 7. Calculate LINEAR Position (x)
        *current_x = *current_phi_rad * WHEEL_RADIUS_R;
    }

    // 8. Update the previous raw position state for the next call
    phi_prev_raw_pos = current_raw_pos;
}
