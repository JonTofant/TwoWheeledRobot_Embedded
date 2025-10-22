/*
 * LQR_Controller.c
 *
 *  Created on: Mar 18, 2025
 *      Author: tofan
 */

#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include "cybergear.h"
#include "kinematics.h"
#include "controler.h"

// PITCH CONTROLLER (PID)
volatile float Kp_pitch = 0.0f;
volatile float Ki_pitch = 0.0f;
volatile float Kd_pitch = 0.0f;

volatile float pitch_integral = 0.0f;
volatile float previous_pitch_error = 0.0f;

volatile float DESIRED_PITCH_RAD = 0.0f;

float MAX_PITCH_HEIGHT_ADJUSTMENT = 20.0f;
extern float base_target_y;
extern float final_y_left;
extern float final_y_right;

void update_pitch_leveling_controller(float current_pitch_rad, float dt)
{
    // 1. Calculate the error
    float pitch_error = DESIRED_PITCH_RAD - current_pitch_rad;

    // 2. Update the integral term (with anti-windup)
    pitch_integral += pitch_error * dt;

    if (Ki_pitch > 0.0f) {
        const float max_integral = MAX_PITCH_HEIGHT_ADJUSTMENT / Ki_pitch;
        if (pitch_integral > max_integral) pitch_integral = max_integral;
        if (pitch_integral < -max_integral) pitch_integral = -max_integral;
    }

    // 3. Derivative term
    float pitch_derivative = 0.0f;
    if (dt > 0.0f) {
        pitch_derivative = (pitch_error - previous_pitch_error) / dt;
    }
    previous_pitch_error = pitch_error;

    // 4. Calculate the controller output (PID)
    float height_adjustment = (Kp_pitch * pitch_error) +
                              (Ki_pitch * pitch_integral) +
                              (Kd_pitch * pitch_derivative);

    // 5. Clamp the final adjustment to prevent sudden, large movements
    if (height_adjustment > MAX_PITCH_HEIGHT_ADJUSTMENT) height_adjustment = MAX_PITCH_HEIGHT_ADJUSTMENT;
    if (height_adjustment < -MAX_PITCH_HEIGHT_ADJUSTMENT) height_adjustment = -MAX_PITCH_HEIGHT_ADJUSTMENT;

    // 6. Apply the adjustment differentially to the base height
    final_y_left  = base_target_y + height_adjustment;
    final_y_right = base_target_y - height_adjustment;

    // 7. Final safety clamp to ensure legs stay within physical limits
    const float LEG_Y_MAX = -13.0f; // Lowest position
    const float LEG_Y_MIN = -24.0f; // Highest position

    if (final_y_left > LEG_Y_MAX) final_y_left = LEG_Y_MAX;
    if (final_y_left < LEG_Y_MIN) final_y_left = LEG_Y_MIN;

    if (final_y_right > LEG_Y_MAX) final_y_right = LEG_Y_MAX;
    if (final_y_right < LEG_Y_MIN) final_y_right = LEG_Y_MIN;
}
