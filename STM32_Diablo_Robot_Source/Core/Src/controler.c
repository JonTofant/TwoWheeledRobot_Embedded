/*
 * LQR_Controller.c
 *
 * Created on: Mar 18, 2025
 * Author: tofan
 */


#include "controler.h"

// PITCH CONTROLLER (PID)
volatile float Kp_pitch = 0.0f;
volatile float Ki_pitch = 0.0f;
volatile float Kd_pitch = 0.0f;


float K_GAINS[2] = {100.0f, 10.0f};
float K_I_THETA = -18.0f; // Integral gain for theta


float Kp_pos_chasis = 0.0;
float Kd_pos_chasis = 0.0f;
float Ki_pos_chasis = 0.0f;

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
 float target_y_chasis = -13.0f;


 // Default values for DDSM115 motor current is 0 [A]
 float current_motor1_out= 0.0f;
 float current_motor2_out= 0.0f;
 float total_torque_out= 0.0f;
 float total_force_out = 0.0f;


 // SMC controller variables
 float lambda_tune  = 1.92f;
 float k1_tune      = 200.0f;
 float k2_tune      = 300.0f;
 float epsilon_tune = 0.1f;
 float L_tune       = 0.0625f;    // ε²
 float gamma_tune   = 0.15f;
 float max_K_tune   = 30.0f;
 float k_tune       = 8.0f;

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
    final_y_left = base_target_y + height_adjustment;
    final_y_right = base_target_y - height_adjustment;

    // 7. Final safety clamp to ensure legs stay within physical limits
    const float LEG_Y_MAX = -13.0f; // Lowest position
    const float LEG_Y_MIN = -24.0f; // Highest position

    if (final_y_left > LEG_Y_MAX) final_y_left = LEG_Y_MAX;
    if (final_y_left < LEG_Y_MIN) final_y_left = LEG_Y_MIN;

    if (final_y_right > LEG_Y_MAX) final_y_right = LEG_Y_MAX;
    if (final_y_right < LEG_Y_MIN) final_y_right = LEG_Y_MIN;
}


void calculate_cascaded_motor_currents(float x_target_left, float x_target_right,
                                       float* current_motor1_out,
                                       float* current_motor2_out,
                                       float* total_force_out)
{
    static float pos_int_L = 0.0f, pos_int_R = 0.0f;
    static float theta_int_L = 0.0f, theta_int_R = 0.0f;
    static uint32_t last_time_ms = 0;

    // ONE SHARED integrator v and gain M – THIS IS WHAT THE PAPER DOES
    static float v = 0.0f;
    static float M = 1.0f;

    uint32_t now = HAL_GetTick();
    float dt = (last_time_ms == 0) ? 0.01f : (now - last_time_ms) / 1000.0f;
    last_time_ms = now;

    float theta     = roll_esp32 - 0.010328498f;
    float theta_dot = gx_esp32;

    // VELOCITY FROM POSITION – NO MORE e-40 GARBAGE
    static float old_xL = 0.0f, old_xR = 0.0f;
    float xL = DDSM115MotorList[0].x;
    float xR = DDSM115MotorList[1].x;
    float x_dot_L_raw = (xL - old_xL) / dt;
    float x_dot_R_raw = (xR - old_xR) / dt;
    old_xL = xL; old_xR = xR;

    // Low-pass
    static float x_dot_L_f = 0.0f, x_dot_R_f = 0.0f;
    x_dot_L_f = 0.8f * x_dot_L_f + 0.2f * x_dot_L_raw;
    x_dot_R_f = 0.8f * x_dot_R_f + 0.2f * x_dot_R_raw;

    float x_dot_L = +x_dot_L_f;   // left forward = positive
    float x_dot_R = -x_dot_R_f;   // right forward = positive

    float vel_err_L = x_dot_L - x_target_left;
    float vel_err_R = x_dot_R - x_target_right;

    // Integral of velocity error
    pos_int_L += vel_err_L * dt;
    pos_int_R += vel_err_R * dt;
    pos_int_L = fmaxf(fminf(pos_int_L, 0.2f), -0.2f);
    pos_int_R = fmaxf(fminf(pos_int_R, 0.2f), -0.2f);

    // Sliding surface PER WHEEL
    float s_L = vel_err_L + lambda_tune * pos_int_L;
    float s_R = vel_err_R + lambda_tune * pos_int_R;

    // SHARED sliding variable (paper uses average)
    float s = 0.5f * (s_L + s_R);
    float abs_s = fabsf(s);
    float sign_s = (s > 0.0f) ? 1.0f : ((s < 0.0f) ? -1.0f : 0.0f);
    float sqrt_s = sqrtf(abs_s);

    // === QUASI-BARRIER FUNCTION – EXACTLY AS IN PAPER ===
    float K;
    if (abs_s > epsilon_tune) {
        // Outside barrier → adaptive gain M
        K = M;
        M += gamma_tune * abs_s * dt;
        if (M > max_K_tune) M = max_K_tune;
        if (M < 1.0f) M = 1.0f;
    } else {
        // Inside barrier → 1/(ε² - s²)  – THIS IS THE KEY
        float den = epsilon_tune*epsilon_tune - s*s;
        if (den <= 0.0f) den = 1e-6f;               // PREVENT DIVISION BY ZERO
        K = L_tune / den;
        if (K > max_K_tune) K = max_K_tune;
        M = 1.0f;                                   // reset adaptation
    }

    // Super-twisting terms – signs proven correct on your robot
    float u_st = k1_tune * K * sqrt_s * sign_s;      // switching
    float v_dot = -k2_tune * K * sign_s;             // integral part
    v += v_dot * dt;
    v = fmaxf(fminf(v, 12.0f), -12.0f);

    float u_sta = u_st + v;

    // Desired tilt – SAME FOR BOTH WHEELS
    float theta_des = (u_sta - lambda_tune * (vel_err_L + vel_err_R) * 0.5f) / k_tune;

    delta_varphi_l = delta_varphi_r = 0.0f;
    theta_des_l_telemetry = theta_des_r_telemetry = theta_des;
    theta_des = fmaxf(fminf(theta_des, 0.2443f), -0.2443f);

    // Inner LQI loop
    float err_L = theta + delta_varphi_l - theta_des;
    float err_R = theta + delta_varphi_r - theta_des;

    theta_int_L += err_L * dt;
    theta_int_R += err_R * dt;
    theta_int_L = fmaxf(fminf(theta_int_L, 0.2f), -0.2f);
    theta_int_R = fmaxf(fminf(theta_int_R, 0.2f), -0.2f);

    float force_L = -(K_GAINS[0] * err_L + K_GAINS[1] * theta_dot + K_I_THETA * theta_int_L);
    float force_R = -(K_GAINS[0] * err_R + K_GAINS[1] * theta_dot + K_I_THETA * theta_int_R);

    if (total_force_out) *total_force_out = 0.5f * (force_L + force_R);

    float curr_L = force_L * WHEEL_RADIUS_R / MOTOR_TORQUE_CONSTANT_KT;
    float curr_R = force_R * WHEEL_RADIUS_R / MOTOR_TORQUE_CONSTANT_KT;

    if (curr_L > 0) curr_L += DZ_LEFT_POS; else if (curr_L < 0) curr_L -= DZ_LEFT_NEG;
    if (curr_R > 0) curr_R += DZ_RIGHT_POS; else if (curr_R < 0) curr_R -= DZ_RIGHT_NEG;

    *current_motor1_out = -curr_R;
    *current_motor2_out =  curr_L;

    // TELEMETRY – now K and v are SHARED
    vel_err_L_telemetry = vel_err_L;
    vel_err_R_telemetry = vel_err_R;
    sliding_surface_L_telemetry = s_L;
    sliding_surface_R_telemetry = s_R;
    controler_output_sta_L_telemetry = u_sta;
    controler_output_sta_R_telemetry = u_sta;
    K_L_telemetry = K_R_telemetry = K;
    v_L_telemetry = v_R_telemetry = v;
}

void posture_controler()
{
float x_L = -DDSM115MotorList[0].x;
float x_dot_L = -DDSM115MotorList[0].x_dot;
float x_ddot_L = -DDSM115MotorList[0].x_ddot; // Fixed: left to left

float x_R = -DDSM115MotorList[1].x; // Fixed: negate for consistent forward=positive
float x_dot_R = -DDSM115MotorList[1].x_dot;
float x_ddot_R = -DDSM115MotorList[1].x_ddot; // Fixed: right to right

float v_L = x_dot_L;
float v_R = x_dot_R;

// Izračun želenega xc z pd regulatorjem
// --- [Outer Loop Errors] ---
float x_err_L = v_L - desired_v_left;
float x_err_R = v_R - desired_v_right;

// --- [Outer Loop Integration (optional)] ---
if (Ki_pos > 0.0f) {
position_integral_L += x_err_L * 0.015; // * dt
position_integral_R += x_err_R * 0.015; // * dt

const float MAX_POS_INTEGRAL = 0.2f;
if (position_integral_L > MAX_POS_INTEGRAL) position_integral_L = MAX_POS_INTEGRAL;
if (position_integral_L < -MAX_POS_INTEGRAL) position_integral_L = -MAX_POS_INTEGRAL;
if (position_integral_R > MAX_POS_INTEGRAL) position_integral_R = MAX_POS_INTEGRAL;
if (position_integral_R < -MAX_POS_INTEGRAL) position_integral_R = -MAX_POS_INTEGRAL;
}

// --- [Theta Desired from Outer Loop PD] ---
xc_des_l = -(Kp_pos_chasis * x_err_L + Kd_pos_chasis * x_ddot_L + Ki_pos_chasis * position_integral_L);
xc_des_r = (Kp_pos_chasis * x_err_R + Kd_pos_chasis * x_ddot_R + Ki_pos_chasis * position_integral_R);

// --- [Clamp Desired Angle] ---
const float MAX_POS_DES = 1.0;
if (xc_des_l > MAX_POS_DES) xc_des_l = MAX_POS_DES;
if (xc_des_l < -MAX_POS_DES) xc_des_l = -MAX_POS_DES;
if (xc_des_r > MAX_POS_DES) xc_des_r = MAX_POS_DES;
if (xc_des_r < -MAX_POS_DES) xc_des_r = -MAX_POS_DES;


float xc_des_l_translated = xc_des_l + 12.9/2;
float xc_des_r_translated = xc_des_r + 12.9/2;

bool success_right = set_leg_foot_position(
&MOTOR_CG_RF, // The "right" motor of the right leg
&MOTOR_CG_RB, // The "left" motor of the right leg
&leg_state_rf,
xc_des_r_translated,
base_target_y
);

// Call the kinematics for the left leg
bool success_left = set_leg_foot_position(
&MOTOR_CG_LF, // The "right" motor of the left leg
&MOTOR_CG_LB, // The "left" motor of the left leg
&leg_state_lf,
xc_des_l_translated,
base_target_y
);

LegGeometryList[0].x_c = xc_des_l_translated;
LegGeometryList[0].y_c = -base_target_y;


LegGeometryList[1].x_c = xc_des_r_translated;
LegGeometryList[1].y_c = -base_target_y;



calculate_L_and_theta(&LegGeometryList[0]);
calculate_L_and_theta(&LegGeometryList[1]);



delta_varphi_l = -LegGeometryList[0].theta;
delta_varphi_r = LegGeometryList[1].theta;

}
