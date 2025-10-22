/*
 * cybergear_angle_protection.c
 *
 *  Created on: Mar 9, 2025
 *      Author: tofan
 */


#include "cybergear_angle_protection.h"

AngleProtectionOutput protect_angle(float current_angle, float target_angle, float min_angle, float max_angle, float soft_margin) {
    AngleProtectionOutput output;
    output.fault = false;

    // First, clamp the desired target angle to within [min_angle, max_angle].
    if (target_angle < min_angle) {
        output.safe_target_angle = min_angle;
        // If the desired target is significantly below the minimum (beyond the soft margin), flag a fault.
        if (target_angle < (min_angle - soft_margin)) {
            output.fault = true;
        }
    } else if (target_angle > max_angle) {
        output.safe_target_angle = max_angle;
        if (target_angle > (max_angle + soft_margin)) {
            output.fault = true;
        }
    } else {
        output.safe_target_angle = target_angle;
    }

    // Next, compute a scaling factor based on the current angle's proximity to the limits.
    // This factor can be used to gradually reduce the control effort as you approach a limit.
    if (current_angle < min_angle + soft_margin) {
        output.scaling_factor = (current_angle - min_angle) / soft_margin;
        if (output.scaling_factor < 0.0f) {
            output.scaling_factor = 0.0f;
        }
    } else if (current_angle > max_angle - soft_margin) {
        output.scaling_factor = (max_angle - current_angle) / soft_margin;
        if (output.scaling_factor < 0.0f) {
            output.scaling_factor = 0.0f;
        }
    } else {
        output.scaling_factor = 1.0f;
    }

    return output;
}

