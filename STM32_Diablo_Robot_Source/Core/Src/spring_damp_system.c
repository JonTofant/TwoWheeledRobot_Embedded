/*
 * spring_damp_system.c
 *
 *  Created on: Mar 8, 2025
 *      Author: tofan
 */

#include <math.h>
#include "spring_damp_system.h"   // Your header with the declaration of compute_desired_current()

// Define some constants (adjust these for your mechanism and hardware):
#define PI 3.14159265358979323846f

// Link lengths (in meters)
static const float l1 = 0.3f;    // length of the first link (thigh)
static const float l2 = 0.3f;    // length of the second link (shank)
// Offset for effective leg length computation (e.g., the horizontal offset)
static const float l0 = 0.1f;

// Desired (equilibrium) leg length (l*) in meters.
static const float l_eq = 0.5f;

// Spring-damper parameters (stiffness in N/m, damping in N*s/m)
static const float K = 100.0f;
static const float B = 10.0f;

// Effective lever arm (in meters) used to convert force into torque.
static const float lever_arm = 0.05f;

// Motor torque constant (k_t) in Nm/A (from the datasheet, e.g., 0.87 Nm/A)
static const float kt = 0.87f;

// Maximum allowable current (A) for safety.
static const float MAX_CURRENT = 3.0f;

// Static variable to store the previous leg length for derivative calculation.
static float l_prev = l_eq;

/**
 * @brief Saturates a given value to the range [-max_val, max_val].
 *
 * @param value The value to be saturated.
 * @param max_val The maximum absolute value allowed.
 * @return The saturated value.
 */
static float saturate(float value, float max_val) {
    if (value > max_val) {
        return max_val;
    } else if (value < -max_val) {
        return -max_val;
    }
    return value;
}

/**
 * @brief Computes the desired currents for two motors acting on one leg.
 *
 * This function calculates the leg's effective length and its rate of change using
 * forward kinematics from the joint angles phi1 and phi2. It then computes the desired
 * force based on a spring-damper model and converts that force into a net torque.
 * Finally, it splits the torque evenly (with opposite signs) between two motors and
 * converts the torque into a current command using the motor torque constant.
 * A saturation block limits the current to a predefined maximum value.
 *
 * @param phi1  Joint angle for motor 1 (in radians).
 * @param phi2  Joint angle for motor 2 (in radians).
 * @param dt    Time step (in seconds) for derivative estimation.
 * @param I_motor1 Pointer to store the desired current for motor 1 (in Amperes).
 * @param I_motor2 Pointer to store the desired current for motor 2 (in Amperes).
 */
void compute_desired_current(float phi1, float phi2, float dt, float* I_motor1, float* I_motor2) {
    // Compute the end-effector (wheel) coordinates using forward kinematics.
    // Standard two-link planar model:
    float x_c = l1 * cosf(phi1) + l2 * cosf(phi1 + phi2);
    float y_c = l1 * sinf(phi1) + l2 * sinf(phi1 + phi2);

    // Compute the effective leg length. Here we use an offset of l0/2 in the x-direction.
    float l = sqrtf((x_c - (l0 / 2.0f)) * (x_c - (l0 / 2.0f)) + y_c * y_c);

    // Estimate the leg extension velocity (l_dot) using a finite difference.
    float l_dot = 0.0f;
    if (dt > 0.0f) {
        l_dot = (l - l_prev) / dt;
    }
    // Update the previous leg length for the next call.
    l_prev = l;

    // Compute the desired force using the spring-damper control law:
    // F_desired = -K * (l - l_eq) - B * l_dot.
    float F_desired = -K * (l - l_eq) - B * l_dot;

    // Convert the desired force into a desired torque using the effective lever arm.
    float tau_desired = F_desired * lever_arm;

    // In a system with two motors working together on one leg,
    // we assume the net torque is split evenly.
    // One motor provides positive torque while the other provides negative torque.
    float tau_motor1 = tau_desired / 2.0f;
    float tau_motor2 = -tau_desired / 2.0f;

    // Convert the torque commands to current commands using the motor's torque constant.
    float current_motor1 = tau_motor1 / kt;
    float current_motor2 = tau_motor2 / kt;

    // Apply current saturation to ensure safety.
    *I_motor1 = saturate(current_motor1, MAX_CURRENT);
    *I_motor2 = saturate(current_motor2, MAX_CURRENT);
}
