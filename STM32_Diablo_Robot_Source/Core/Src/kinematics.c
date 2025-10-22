/*
 * kinematics.c
 *
 *  Created on: Mar 10, 2025
 *      Author: tofan
 */

#include "kinematics.h"
#include <math.h>


//Inverzna kinematika
const float L1_C = 12.9f;
const float L2_C = 10.0f;
const float L3_C = 20.0f;
const float PI_C = 3.14159265359f;

// Header Kinematika ?
float base_target_y = -13.0f;
float final_y_left = -13.0f;
float final_y_right = -13.0f;

// Header kinematika ?
float xc_des_l =0;
float xc_des_r = 0;

float delta_varphi_l;
float delta_varphi_r;

// Leg geometry definition
LegGeometry LegGeometryList[MAX_NUM_LEGS] = {
    {
        .l0 = 12.9f, .l1 = 10.0f, .l2 = 20.0f,
        .l = 0.0f, .l_dot = 0.0f, .r_eff = 0.0f,
        .l_prev = 0.0f, .x_c = 0.0f, .y_c = 0.0f,
        .theta = 0.0f, .l_eq = 0.13f, .l_dot_desired = 0.0f,
		.theta_ref = 0.0f, .theta_dot = 0.0f, .theta_prev = 0.0f,
        .l_ddot = 0.0f, .l_dot_prev = 0.0f,
        .J = {{0.0f, 0.0f}, {0.0f, 0.0f}},
        .J_T = {{0.0f, 0.0f}, {0.0f, 0.0f}}
    },
    {
        .l0 = 12.9f, .l1 = 10.0f, .l2 = 20.0f,
        .l = 0.0f, .l_dot = 0.0f, .r_eff = 0.0f,
        .l_prev = 0.0f, .x_c = 0.0f, .y_c = 0.0f,
		.theta_ref = 0.0f, .theta_dot = 0.0f, .theta_prev = 0.0f,
        .theta = 0.0f, .l_eq = 0.13f, .l_dot_desired = 0.0f,
        .l_ddot = 0.0f, .l_dot_prev = 0.0f,
        .J = {{0.0f, 0.0f}, {0.0f, 0.0f}},
        .J_T = {{0.0f, 0.0f}, {0.0f, 0.0f}}
    }
};


// Defintion of the r_eff function
// r_eff = l1 * sin(theta1) + l2 * sin(theta1 + theta2)
void calculate_r_eff(float theta1, float theta2, LegGeometry* legGeometry) {
	// theta 1 is offset by 90 degrees
	float theta1_c = theta1;
	float theta2_c = theta2;

	// Calculate the effective leg length
	legGeometry->r_eff = legGeometry->l1 * sinf(theta1_c) + legGeometry->l2 * sinf(theta1_c + theta2_c);
}

// Function to calculate L and L_dot

void calculate_L_and_L_dot(float theta1, float theta2, LegGeometry* legGeometry, float dt) {
	float theta1_c =theta1;
	float theta2_c =theta2;
    // Calculate x_c and y_c based on the angles and leg segments.
    legGeometry->x_c = legGeometry->l1 * cosf(theta1_c) + legGeometry->l2 * cosf(theta1_c + theta2_c);
    legGeometry->y_c = legGeometry->l1 * sinf(theta1_c) + legGeometry->l2 * sinf(theta1_c + theta2_c);

    // Correct calculation of L: square the difference and add y_c squared, then take the square root.
    legGeometry->l = sqrtf((legGeometry->x_c - (legGeometry->l0 / 2.0f)) * (legGeometry->x_c - (legGeometry->l0 / 2.0f)) + legGeometry->y_c * legGeometry->y_c);

    // Compute the derivative of L, making sure dt is positive.
    legGeometry->l_dot = (dt > 0.0f) ? (legGeometry->l - legGeometry->l_prev) / dt : 0.0f;

    // Update l_prev for the next iteration.
    legGeometry->l_prev = legGeometry->l;
}


// Function to calculate Xc and Yc
void calculate_Xc_Yc(float phi1, float phi2, LegGeometry* legGeometry) {
	// Calculate x_c and y_c based on the angles and leg segments.
	phi1 =  phi1;
	phi2 = phi2;
	float l0 = legGeometry->l0;
	float l1 = legGeometry->l1;
	float l2 = legGeometry->l2;
	float x_c = legGeometry->x_c;
	float y_c = legGeometry->y_c;

	float PHI = sqrtf((4*l2*l2)/(l0*l0-2*l1*l1*cosf(phi1-phi2)+2*l1*l1-2*l0*l1*(cosf(phi1)-cosf(phi2)))-1);
	x_c =  0.5f * (
		    l0
		    + l1 * (cosf(phi1) + cosf(phi2))
		    + l1 * PHI * (sinf(phi1) - sinf(phi2))
		);
	y_c = 0.5f * (
	    l1 * (sinf(phi1) + sinf(phi2))   	  // l1(sin(phi1) + sin(phi2))
	  + l0 * PHI                        	  // + l0 * PHI
	  - l1 * PHI * (cosf(phi1) + cosf(phi2))  // - l1*PHI*(cos(phi1)+cos(phi2))
	);

	// Write the results back to the leg geometry struct
	legGeometry->x_c = x_c;
	legGeometry->y_c = y_c;

}
// This is the corrected version of your function.
// It fixes the typo (0.05f -> 0.5f) and uses the safe atan2f function.
void calculate_L_and_theta(LegGeometry* legGeometry) {
	// --- Input values from the struct ---
	// We don't need to declare local variables like 'l' and 'theta'
	// because we will write directly to the struct at the end.
	const float l0 = legGeometry->l0;   // Base link distance (e.g., 0.129)
	const float x_c = legGeometry->x_c; // Absolute X position of the foot
	const float y_c = legGeometry->y_c; // Absolute Y position of the foot

	// --- Define the origin of the virtual leg ---
	// This is the midpoint of the base link.
	const float virtual_leg_base_x = l0 / 2.0f;
	const float virtual_leg_base_y = 0.0f;

	// --- Calculate the vector from the virtual leg's base to the foot ---
	//float delta_x = x_c - virtual_leg_base_x;
	//float delta_y = y_c - virtual_leg_base_y;

	// --- Calculate and write the results back to the leg geometry struct ---

	// Calculate the virtual leg length 'l'
	// Calculate the virtual leg angle 'theta' using the correct math
	legGeometry->theta = atanf((x_c - 0.5f*l0)/y_c);
}
void calculate_L_dot(LegGeometry* legGeometry, float dt) {
	// Calculate the derivative of L, making sure dt is positive.
	float l = legGeometry->l;
	float l_prev = legGeometry->l_prev;
	float l_dot = legGeometry->l_dot;
	l_dot = (dt > 0.0f) ? (l - l_prev) / dt : 0.0f;
	// Update l_prev for the next iteration.
	legGeometry->l_dot = l_dot;
	legGeometry->l_prev = l;
}

void calculate_L_ddot(LegGeometry* legGeometry, float dt) {
	// Calculate the derivative of L, making sure dt is positive.
	float l_dot = legGeometry->l_dot;
	float l_dot_prev = legGeometry->l_dot_prev;
	float l_ddot = legGeometry->l_ddot;
	l_ddot = (dt > 0.0f) ? (l_dot - l_dot_prev) / dt : 0.0f;
	// Update l_prev for the next iteration.
	legGeometry->l_ddot = l_ddot;
	legGeometry->l_dot_prev = l_dot;
}

void calculate_Transpose_Jaboian(LegGeometry* legGeometry, float phi1, float phi2) {
	// Calculate the transpose of the Jacobian
	float l1 = legGeometry->l1;
	float l2 = legGeometry->l2;
	float l0 = legGeometry->l0;
	float x0  = sinf(phi1);
	float x1  = cosf(phi1);
	float x2  = pow(l2, 2);
	float x3  = 2*pow(l1, 2);
	float x4  = phi1 - phi2;
	float x5  = 2*l0*l1;
	float x6  = pow(l0, 2) - x3*cosf(x4) + x3 - x5*(x1 - cosf(phi2));
	float x7  = sqrt(4*x2/x6 - 1);
	float dxc_dphi1 = -0.5*l1*x0 + 0.5*l1*x1*x7 - 1.0*l1*x2*(x0 - sinf(phi2))*(x0*x5 + x3*sinf(x4))/(pow(x6, 2)*x7);
	x0  = sinf(phi1);
	x1  = cosf(phi1);
	x2  = pow(l2, 2);
	x3  = 2*pow(l1, 2);
	x4  = phi1 - phi2;
	x5  = 2*l0*l1;
	x6  = pow(l0, 2) - x3*cosf(x4) + x3 - x5*(x1 - cosf(phi2));
	x7  = sqrt(4*x2/x6 - 1);
	float dxc_dphi2 = -0.5*l1*x0 + 0.5*l1*x1*x7 - 1.0*l1*x2*(x0 - sinf(phi2))*(x0*x5 + x3*sinf(x4))/(pow(x6, 2)*x7);
	x0 =  pow(l0, 2);
	x1 =  2*pow(l1, 2);
	x2 = phi1 - phi2;
	x3 = x1*cosf(x2);
	x4 = cosf(phi1);
	x5 =  cosf(phi2);
	x6 =  2*l0*l1*(x4 - x5);
	x7 =  x0 + x1 - x3 - x6;
	float x8 = pow(x7, 2);
	float x9 =  pow(l2, 2);
	float x10 =  sqrt((-x0 - x1 + x3 + x6 + 4*x9)/x7);
	float x11 =  sinf(phi1);
	float x12 =  2.0*x9*(l0*x11 + l1*sinf(x2));
	float dyc_dphi1 = l1*(-l0*x12 + l1*x12*(x4 + x5) + 0.5*x10*x8*(x10*x11 + x4))/(x10*x8);
	x0 =pow(l0, 2);
	x1 = 2*pow(l1, 2);
	x2 = phi1 - phi2;
	x3 =  x1*cos(x2);
	x4 =  cos(phi1);
	x5 =  cos(phi2);
	x6 =  2*l0*l1*(x4 - x5);
	x7 =  x0 + x1 - x3 - x6;
	x8 = pow(x7, 2);
	x9 =  pow(l2, 2);
	x10 = sqrt((-x0 - x1 + x3 + x6 + 4*x9)/x7);
	x11 =  sin(phi2);
	x12 = 2.0*x9*(l0*x11 + l1*sin(x2));
	float dyc_dphi2 = l1*(l0*x12 - l1*x12*(x4 + x5) + 0.5*x10*x8*(x10*x11 + x5))/(x10*x8);
	float J[2][2] = {
		{dxc_dphi1, dxc_dphi2 },
		{dyc_dphi1, dyc_dphi2 }
	};
	// Calculate the transpose of the Jacobian
	float J_T[2][2] = {
		{J[0][0], J[1][0]},
		{J[0][1], J[1][1]}
	};

	// Write the results of jacobian and jacobian transpose back to the leg geometry struct
	legGeometry->J[0][0] = J[0][0];
	legGeometry->J[0][1] = J[0][1];
	legGeometry->J[1][0] = J[1][0];
	legGeometry->J[1][1] = J[1][1];

	legGeometry->J_T[0][0] = J_T[0][0];
	legGeometry->J_T[0][1] = J_T[0][1];
	legGeometry->J_T[1][0] = J_T[1][0];
	legGeometry->J_T[1][1] = J_T[1][1];

}

void calulate_Theta_dot(LegGeometry* legGeometry, float dt) {
	// Calculate the derivative of theta, making sure dt is positive.
	float theta = legGeometry->theta;
	float theta_prev = legGeometry->theta_prev;
	float theta_dot = legGeometry->theta_dot;
	theta_dot = (dt > 0.0f) ? (theta - theta_prev) / dt : 0.0f;
	// Update l_prev for the next iteration.
	legGeometry->theta_dot = theta_dot;
	legGeometry->theta_prev = theta;
}


