/*
 * LQR_Controller.c
 *
 *  Created on: Mar 18, 2025
 *      Author: tofan
 */


#include "controler.h"

//SMC
float lambda =2.0f;   // Convergence speed parameter
float K = 0.1f;       // SMC gain (must exceed max expected disturbance)
float phi = 3.5;     // Boundary layer for chattering reduction
// Tunable ASMC gains (exposed globally so you can tune at runtime)
// K_sigma acts on sigma (primary switching); K_s acts on s (secondary damping)
float K_sigma = 0.08f; // main ASMC switching gain (tunable)
float K_s     = 0.02f; // secondary proportional on s (damping / smoothness)


float sliding_surface_L;
float sliding_surface_R;
float sliding_surface_dot_L;
float sliding_surface_dot_R;
float sigma_L;
float sigma_R;
float alpha_sigma = 30.0f; // convergence rate for sigma -> increases speed to drive s -> 0



// PITCH CONTROLLER (PID)
volatile float Kp_pitch = 0.0f;
volatile float Ki_pitch = 0.0f;
volatile float Kd_pitch = 0.0f;


// Event based variables
float event_threshold = 1.5f;        // tunable
float max_error_angle = 1.5f;        // experimental normalization constants
float max_error_angvel = 100.0f;
float max_error_vel = 13.0f;
volatile float dt = 0.0f;

volatile uint32_t event_trigger_count = 0;
volatile uint32_t event_trigger_window_start = 0;
float avg_trigger_rate_hz = 0.0f;

float e_angle = 0;
float e_angvel = 0;
float e_vel    = 0;

float e_norm = 0;




float K_GAINS[2] = {130.0f,   4.0f};
float K_I_THETA = -12.0f; // Integral gain for theta


float Kp_pos_chasis = 0.0f;
float Kd_pos_chasis = -0.0;
float Ki_pos_chasis = 0.0f;


float position_integral_L;
float position_integral_R;

// Position control gains for fall strategy
float Kp_pos = 0.13f;
float Kd_pos = -0.0112;
float Ki_pos = 0.25f;

float d_e_angle_threshold = 0.5f;   // Threshold for angular error change (rad/s)
float d_e_angvel_threshold = 1.8f;  // Threshold for angular velocity error change (rad/s^2)
float d_e_vel_threshold = 3.5f;     // Threshold for linear velocity error change (m/s^2)


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

/**
 * @brief  Calculates motor currents for a cascaded control of inverted pendulum.
 *         Outer loop is velocity control using ASMC sliding surfaces computed externally.
 *         Inner loop is angle tracking via LQI.
 *
 * @param  x_target_left   Desired linear velocity for left wheel [m/s]
 * @param  x_target_right  Desired linear velocity for right wheel [m/s]
 * @param  sliding_surface_left  Precomputed sliding surface for left wheel (from 2 ms loop)
 * @param  sliding_surface_right Precomputed sliding surface for right wheel (from 2 ms loop)
 * @param  current_motor1_out    Pointer to output current for right motor [A]
 * @param  current_motor2_out    Pointer to output current for left motor [A]
 * @param  total_force_out       Optional pointer to average total force [N]
 */
void calculate_cascaded_motor_currents_smc(float x_target_left, float x_target_right,
                                           float* current_motor1_out,
                                           float* current_motor2_out,
                                           float* total_force_out)
{
    static float theta_error_integral_left  = 0.0f;
    static float theta_error_integral_right = 0.0f;
    static uint32_t last_cycles = 0;

    // --- Compute dt dynamically ---
    uint32_t current_cycles = DWT->CYCCNT;
    dt = (current_cycles - last_cycles) / (float)SystemCoreClock;
    last_cycles = current_cycles;

    // --- Sensor input ---
    float current_theta     = roll_esp32 - 0.010328498f;
    float current_theta_dot = gx_esp32;

    // --- Inner loop LQI ---
    float theta_error_left  = current_theta + delta_varphi_l - theta_des_l_telemetry;
    float theta_error_right = current_theta + delta_varphi_r - theta_des_r_telemetry;

    theta_error_integral_left  += theta_error_left  * dt;
    theta_error_integral_right += theta_error_right * dt;

    // Limit integral
    const float MAX_THETA_I = 0.2f;
    if (theta_error_integral_left  > MAX_THETA_I) theta_error_integral_left  = MAX_THETA_I;
    if (theta_error_integral_left  < -MAX_THETA_I) theta_error_integral_left  = -MAX_THETA_I;
    if (theta_error_integral_right > MAX_THETA_I) theta_error_integral_right = MAX_THETA_I;
    if (theta_error_integral_right < -MAX_THETA_I) theta_error_integral_right = -MAX_THETA_I;

    // Compute LQI force
    float force_left  = -(K_GAINS[0] * theta_error_left  + K_GAINS[1] * current_theta_dot + K_I_THETA * theta_error_integral_left);
    float force_right = -(K_GAINS[0] * theta_error_right + K_GAINS[1] * current_theta_dot + K_I_THETA * theta_error_integral_right);

    if (total_force_out)
        *total_force_out = 0.5f * (force_left + force_right);

    float current_left  = (force_left  * WHEEL_RADIUS_R) / MOTOR_TORQUE_CONSTANT_KT;
    float current_right = (force_right * WHEEL_RADIUS_R) / MOTOR_TORQUE_CONSTANT_KT;

    // Deadzone compensation
    if (current_left > 0) current_left  += DZ_LEFT_POS;
    else if (current_left < 0) current_left  -= DZ_LEFT_NEG;

    if (current_right > 0) current_right += DZ_RIGHT_POS;
    else if (current_right < 0) current_right -= DZ_RIGHT_NEG;

    // Output currents
    *current_motor1_out = -current_right;
    *current_motor2_out =  current_left;
}

