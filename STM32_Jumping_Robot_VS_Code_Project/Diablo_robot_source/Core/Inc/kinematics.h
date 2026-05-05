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
#include <stdbool.h>

//Inverzna kinematika
extern const float L1_C;
extern const float L2_C;
extern const float L3_C;
extern const float PI_C;

// Header Kinematika ?
extern float base_target_y;
extern float final_y_left;
extern float final_y_right;

// Header kinematika ?
extern float xc_des_l;
extern float xc_des_r;

extern float delta_varphi_l;
extern float delta_varphi_r;

// Define the struct and typedef
typedef struct {
    bool is_initialized;
    float prev_B_x;
    float prev_B_y;
    float prev_C_x;
    float prev_C_y;
} LegState;

// Declare globals if you want
extern LegState leg_state_rf;
extern LegState leg_state_lf;
extern LegState leg_state_rb;
extern LegState leg_state_lb;

// Function prototypes
void init_leg_state(LegState* state);
bool set_leg_foot_position(CyberGear* motor_right, CyberGear* motor_left,
                           LegState* leg_state, float xf, float yf);

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


void calculate_L_and_theta(LegGeometry* legGeometry);


bool solveIKTwoSolutions_c(float base_x, float base_y, float foot_x, float foot_y,
                                  float L_upper, float L_lower,
                                  float *out_x1, float *out_y1, float *out_x2, float *out_y2);

void chooseContinuousSolution_c(float prev_x, float prev_y,
                                       float x1, float y1, float x2, float y2,
                                       float *out_x, float *out_y);
#endif /* INC_KINEMATICS_H_ */
