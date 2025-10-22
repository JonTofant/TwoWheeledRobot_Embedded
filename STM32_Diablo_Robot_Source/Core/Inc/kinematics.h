/*
 * kinematics.h
 *
 *  Created on: Mar 10, 2025
 *      Author: tofan
 */

#ifndef INC_KINEMATICS_H_
#define INC_KINEMATICS_H_

#define MAX_NUM_LEGS 2

// Include cybergear.h for the CyberGear struct definition
#include "cybergear.h"

// Struct for leg geometry
typedef struct {
	float l0;  	 // Length of the hip-to-hip distance
	float l1;  	 // Length of the upper leg segment
	float l2;  	 // Length of the lower leg segment
	float l;   	 // Length of L perpendicular to the ground
	float l_dot; // Derivative of l
	float r_eff; // Effective leg length
	float l_prev; // Previous value of l
	float x_c;   // x coordinate of the foot
	float y_c;   // y coordinate of the foot
	float theta; // Angle of the leg
	float l_eq;  // Equilibrium length
	float l_dot_desired; // Desired l_dot
	float l_ddot; // Desired l_ddot
	float l_dot_prev; // Previous value of l_ddot
	float theta_ref; // Reference angle
	float theta_dot; // Angle velocity
	float theta_prev; // Previous angle
	float J[2][2]; // Jacobian
	float J_T[2][2]; // Transpose of the Jacobian
} LegGeometry;

// External declaration of the leg geometry
extern LegGeometry LegGeometryList[MAX_NUM_LEGS];

// r_eff function prototype
// Input: theta1, theta2 (in radians), and pointer to LegGeometry struct
// Output: r_eff (in meters)
void calculate_r_eff(float theta1, float theta2, LegGeometry* legGeometry);
void calculate_L_and_L_dot(float theta1, float theta2, LegGeometry* legGeometry, float dt);
// Function to calculate Xc and Yc
void calculate_Xc_Yc(float phi1, float phi2, LegGeometry* legGeometry);
void calculate_L_and_theta(LegGeometry* legGeometry);
void calculate_L_dot(LegGeometry* legGeometry, float dt);
void calculate_L_ddot(LegGeometry* legGeometry, float dt);
void calculate_Transpose_Jaboian(LegGeometry* legGeometry, float phi1, float phi2);
void calulate_Theta_dot(LegGeometry* legGeometry, float dt);
#endif /* INC_KINEMATICS_H_ */
