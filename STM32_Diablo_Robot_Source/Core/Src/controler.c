/*
 * LQR_Controller.c
 *
 *  Created on: Mar 18, 2025
 *      Author: tofan
 */


#include "controler.h"

//SMC
float lambda =2.3f;   // Convergence speed parameter
float K = 0.04f;       // SMC gain (must exceed max expected disturbance)
float phi = 0.04;     // Boundary layer for chattering reduction
float sliding_surface_L;
float sliding_surface_R;

// PITCH CONTROLLER (PID)
volatile float Kp_pitch = 0.0f;
volatile float Ki_pitch = 0.0f;
volatile float Kd_pitch = 0.0f;


// Event based variables
float event_threshold = 0.1f;        // tunable
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




float K_GAINS[2] = {100.0f,   10.0f};
float K_I_THETA = -18.0f; // Integral gain for theta


float Kp_pos_chasis = 1.2f;
float Kd_pos_chasis = -0.112;
float Ki_pos_chasis = 0.6f;


float position_integral_L;
float position_integral_R;

// Position control gains for fall strategy
float Kp_pos = 0.13f;
float Kd_pos = -0.0112;
float Ki_pos = 0.25f;

extern float d_e_angle_threshold = 0.5f;   // Threshold for angular error change (rad/s)
extern float d_e_angvel_threshold = 2.0f;  // Threshold for angular velocity error change (rad/s^2)
extern float d_e_vel_threshold = 1.0f;     // Threshold for linear velocity error change (m/s^2)


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
 *         Outer loop is velocity control using SMC sliding surfaces computed externally.
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
                                           float sliding_surface_left, float sliding_surface_right,
                                           float* current_motor1_out,
                                           float* current_motor2_out,
                                           float* total_force_out)
{
    // --- [STATE] Persistent Integrators for Inner Loop LQI ---
    static float theta_error_integral_left  = 0.0f;
    static float theta_error_integral_right = 0.0f;
    static uint32_t last_cycles = 0;


    // --- [TIMING] Compute dt in seconds ---
     uint32_t current_cycles = DWT->CYCCNT;
     dt = (current_cycles - last_cycles) / (float)SystemCoreClock;
     last_cycles = current_cycles;

    // --- [SENSOR INPUT] IMU & Wheel States ---
    float current_theta     = roll_esp32 - 0.010328498f;  // Corrected offset
    float current_theta_dot = gx_esp32;

    float wheel_vel_left  = -DDSM115MotorList[0].x_dot;
    float wheel_vel_right =  DDSM115MotorList[1].x_dot;
    float wheel_acc_left  = -DDSM115MotorList[0].x_ddot;
    float wheel_acc_right =  DDSM115MotorList[1].x_ddot;

    ////////////////////////////////////////////////////////////////////////////////
    // -------------------- OUTER LOOP: SLIDING SURFACE -> Theta Desired -------- //
    ////////////////////////////////////////////////////////////////////////////////

    // --- Saturation function for chattering reduction ---
    float sat_left  = sliding_surface_left  / phi;
    float sat_right = sliding_surface_right / phi;

    if (sat_left  >  1.0f) sat_left  =  1.0f;
    if (sat_left  < -1.0f) sat_left  = -1.0f;
    if (sat_right >  1.0f) sat_right =  1.0f;
    if (sat_right < -1.0f) sat_right = -1.0f;

    // --- Desired pitch angle for inner loop LQI ---
    float theta_des_left  = -K * sat_left;
    float theta_des_right = -K * sat_right;

    // --- Clamp Desired Angle to safe limits ---
    const float MAX_THETA_DES = 0.244346095f;  // ≈ 14 degrees
    if (theta_des_left  > MAX_THETA_DES) theta_des_left  = MAX_THETA_DES;
    if (theta_des_left  < -MAX_THETA_DES) theta_des_left  = -MAX_THETA_DES;
    if (theta_des_right > MAX_THETA_DES) theta_des_right = MAX_THETA_DES;
    if (theta_des_right < -MAX_THETA_DES) theta_des_right = -MAX_THETA_DES;

    ////////////////////////////////////////////////////////////////////////////////
    // -------------------- INNER LOOP: ANGLE TRACKING LQI ----------------------- //
    ////////////////////////////////////////////////////////////////////////////////

    float theta_error_left  = current_theta + delta_varphi_l - theta_des_left;
    float theta_error_right = current_theta + delta_varphi_r - theta_des_right;

    // --- Integrate inner-loop error ---
    theta_error_integral_left  += theta_error_left  * dt;
    theta_error_integral_right += theta_error_right * dt;

    // --- Limit integral term to prevent windup ---
    const float MAX_THETA_I = 0.2f;
    if (theta_error_integral_left  > MAX_THETA_I) theta_error_integral_left  = MAX_THETA_I;
    if (theta_error_integral_left  < -MAX_THETA_I) theta_error_integral_left  = -MAX_THETA_I;
    if (theta_error_integral_right > MAX_THETA_I) theta_error_integral_right = MAX_THETA_I;
    if (theta_error_integral_right < -MAX_THETA_I) theta_error_integral_right = -MAX_THETA_I;

    // --- Compute motor forces from LQI ---
    float force_left  = -(K_GAINS[0] * theta_error_left  + K_GAINS[1] * current_theta_dot + K_I_THETA * theta_error_integral_left);
    float force_right = -(K_GAINS[0] * theta_error_right + K_GAINS[1] * current_theta_dot + K_I_THETA * theta_error_integral_right);

    // --- Optional: log average total force ---
    if (total_force_out) {
        *total_force_out = 0.5f * (force_left + force_right);
    }

    // --- Convert force to motor currents ---
    float current_left  = (force_left  * WHEEL_RADIUS_R) / MOTOR_TORQUE_CONSTANT_KT;
    float current_right = (force_right * WHEEL_RADIUS_R) / MOTOR_TORQUE_CONSTANT_KT;

    // --- Deadzone Compensation ---
    if (current_left > 0) current_left  += DZ_LEFT_POS;
    else if (current_left < 0) current_left  -= DZ_LEFT_NEG;

    if (current_right > 0) current_right += DZ_RIGHT_POS;
    else if (current_right < 0) current_right -= DZ_RIGHT_NEG;

    // --- Final motor outputs ---
    *current_motor1_out = -current_right;  // Right motor
    *current_motor2_out =  current_left;   // Left motor
}


