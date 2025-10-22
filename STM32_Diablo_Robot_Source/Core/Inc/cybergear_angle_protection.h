/*
 * cybergear_angle_protection.h
 *
 *  Created on: Mar 9, 2025
 *      Author: tofan
 */

#ifndef INC_CYBERGEAR_ANGLE_PROTECTION_H_
#define INC_CYBERGEAR_ANGLE_PROTECTION_H_

#include <stdbool.h>

// Structure for angle protection output.
typedef struct {
    float safe_target_angle;  // The target angle adjusted (clamped) to safe bounds.
    float scaling_factor;     // Factor between 0 and 1 that scales control effort near the limits.
    bool fault;               // True if the target is far outside the safe range.
} AngleProtectionOutput;

/**
 * @brief Adjusts the desired angle command to ensure it remains within safe limits.
 *
 * The function clamps the target angle to the interval [min_angle, max_angle].
 * It also calculates a scaling factor that decreases as the current angle approaches
 * either limit (within a "soft margin"). If the desired target is far outside the safe
 * bounds, it sets a fault flag.
 *
 * @param current_angle  The current measured angle (in radians).
 * @param target_angle   The desired target angle (in radians).
 * @param min_angle      The minimum allowed angle (in radians).
 * @param max_angle      The maximum allowed angle (in radians).
 * @param soft_margin    The margin (in radians) from the limits where control effort should be scaled down.
 * @return AngleProtectionOutput Structure containing the safe target angle, scaling factor, and fault flag.
 */
AngleProtectionOutput protect_angle(float current_angle, float target_angle, float min_angle, float max_angle, float soft_margin);


#endif /* INC_CYBERGEAR_ANGLE_PROTECTION_H_ */
