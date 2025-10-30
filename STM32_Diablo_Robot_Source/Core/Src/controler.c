/*
 * LQR_Controller.c
 *
 *  Created on: Mar 18, 2025
 *      Author: tofan
 */


#include "controler.h"

// PITCH CONTROLLER (PID)
volatile float Kp_pitch = 0.0f;
volatile float Ki_pitch = 0.0f;
volatile float Kd_pitch = 0.0f;


float K_GAINS[2] = {100.0f,   10.0f};
float K_I_THETA = -18.0f; // Integral gain for theta


float Kp_pos_chasis = 1.2f;
float Kd_pos_chasis = -0.15f;
float Ki_pos_chasis = 0.6f;

float position_integral_L;
float position_integral_R;

// Position control gains for fall strategy
 float Kp_pos = 0.13f;
 float Kd_pos = -0.015f;
 float Ki_pos = 0.25f;

 float desired_v_left=0;
 float desired_v_right=0;

 //Target chasis position
 float target_x_chasis = 12.9/2;
 float target_y_chasis = -13.0;


 // Default values for DDSM115 motor current is 0 [A]
 float current_motor1_out= 0.0f;
 float current_motor2_out= 0.0f;
 float total_torque_out= 0.0f;
 float total_force_out = 0.0f;


float lambda =2.3f;   // Convergence speed parameter
float K = 0.04f;       // SMC gain (must exceed max expected disturbance)
float phi = 0.04;     // Boundary layer for chattering reduction


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


//////////////////////////////////////////////////////////////////////////////
// -------------------- CASCADED MOTOR CONTROL: SMC OUTER LOOP ------------- //
//////////////////////////////////////////////////////////////////////////////

void calculate_cascaded_motor_currents(float x_target_left, float x_target_right,
                                       float* current_motor1_out,
                                       float* current_motor2_out,
                                       float* total_force_out)
{
    // --- [STATE] Persistent Integrators for Inner Loop LQI ---
    static float theta_error_integral_L = 0.0f;
    static float theta_error_integral_R = 0.0f;
    static uint32_t last_time_ms = 0;

    // --- [TIMING] Compute dt ---
    uint32_t current_time_ms = HAL_GetTick();
    float dt = (last_time_ms == 0) ? 0.01f : (current_time_ms - last_time_ms) / 1000.0f;
    last_time_ms = current_time_ms;

    // --- [SENSOR INPUT] IMU & Wheel States ---
    float temp_theta     = roll_esp32 - 0.010328498f;  // Corrected offset
    float temp_theta_dot = gx_esp32;

    float x_dot_L = -DDSM115MotorList[0].x_dot;
    float x_dot_R =  DDSM115MotorList[1].x_dot;
    float x_ddot_L = -DDSM115MotorList[0].x_ddot;
    float x_ddot_R =  DDSM115MotorList[1].x_ddot;

    ////////////////////////////////////////////////////////////////////////////////
    // -------------------- OUTER LOOP: VELOCITY CONTROL (SMC) ------------------ //
    ////////////////////////////////////////////////////////////////////////////////

    // 1. Define velocity errors (desired minus actual)
    float e_v_L = x_dot_L - x_target_left;
    float e_v_R = x_dot_R - x_target_right;

    // 2. Sliding Surface (with integral term for zero steady-state error)
    //    s = e_v + lambda * integral(e_v)
    static float integral_e_v_L = 0.0f;
    static float integral_e_v_R = 0.0f;

    // float lambda   // Convergence speed parameter
    // float K        // SMC gain (must exceed max expected disturbance)
    // float phi      // Boundary layer for chattering reduction

    integral_e_v_L += e_v_L * dt;
    integral_e_v_R += e_v_R * dt;

    float s_L = e_v_L + lambda * integral_e_v_L;
    float s_R = e_v_R + lambda * integral_e_v_R;

    // 3. Saturation function to reduce chattering
    //    sat(x) = -1   if x < -1
    //             x    if -1 <= x <= 1
    //             1    if x > 1
    //
    //     Graphic:
    //       1 |        ______
    //         |       /
    //         |      /
    //       0 |-----/-------
    //         |    /
    //         |   /
    //      -1 |__/
    //
    float sat_L = s_L / phi;
    if (sat_L > 1.0f) sat_L = 1.0f;
    else if (sat_L < -1.0f) sat_L = -1.0f;

    float sat_R = s_R / phi;
    if (sat_R > 1.0f) sat_R = 1.0f;
    else if (sat_R < -1.0f) sat_R = -1.0f;

    // 4. Compute desired theta from SMC output
    float theta_des_L = -K * sat_L;
    float theta_des_R = -K * sat_R;

    // --- [Clamp Desired Angle] ---
    const float MAX_THETA_DES = 0.244346095f;  // ≈ 14 degrees
    if (theta_des_L > MAX_THETA_DES) theta_des_L = MAX_THETA_DES;
    if (theta_des_L < -MAX_THETA_DES) theta_des_L = -MAX_THETA_DES;
    if (theta_des_R > MAX_THETA_DES) theta_des_R = MAX_THETA_DES;
    if (theta_des_R < -MAX_THETA_DES) theta_des_R = -MAX_THETA_DES;

    // --- [Inner Loop: Angle Tracking LQI] ---
    float temp_theta_l_plus_varphi = temp_theta + delta_varphi_l;
    float temp_theta_r_plus_varphi = temp_theta + delta_varphi_r;

    float theta_error_L = temp_theta_l_plus_varphi - theta_des_L;
    float theta_error_R = temp_theta_r_plus_varphi - theta_des_R;

    theta_error_integral_L += theta_error_L * dt;
    theta_error_integral_R += theta_error_R * dt;

    const float MAX_THETA_I = 0.2f;
    if (theta_error_integral_L > MAX_THETA_I) theta_error_integral_L = MAX_THETA_I;
    if (theta_error_integral_L < -MAX_THETA_I) theta_error_integral_L = -MAX_THETA_I;
    if (theta_error_integral_R > MAX_THETA_I) theta_error_integral_R = MAX_THETA_I;
    if (theta_error_integral_R < -MAX_THETA_I) theta_error_integral_R = -MAX_THETA_I;

    float force_L = -(K_GAINS[0] * theta_error_L + K_GAINS[1] * temp_theta_dot + K_I_THETA * theta_error_integral_L);
    float force_R = -(K_GAINS[0] * theta_error_R + K_GAINS[1] * temp_theta_dot + K_I_THETA * theta_error_integral_R);

    if (total_force_out) *total_force_out = 0.5f * (force_L + force_R);

    // --- [Actuator Conversion] ---
    float current_L = (force_L * WHEEL_RADIUS_R) / MOTOR_TORQUE_CONSTANT_KT;
    float current_R = (force_R * WHEEL_RADIUS_R) / MOTOR_TORQUE_CONSTANT_KT;

    // --- [Deadzone Compensation] ---
    if (current_L > 0) current_L += DZ_LEFT_POS;
    else if (current_L < 0) current_L -= DZ_LEFT_NEG;

    if (current_R > 0) current_R += DZ_RIGHT_POS;
    else if (current_R < 0) current_R -= DZ_RIGHT_NEG;

    *current_motor1_out = -current_R;  // Right motor
    *current_motor2_out =  current_L;  // Left motor
}

