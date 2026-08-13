/*
 * kinematics.c
 *
 *  Created on: Mar 10, 2025
 *      Author: tofan
 */

#include "kinematics.h"

// Base leg height target [cm]. Written by joystick.c's dpad handler and
// JumpStrategy.c's jump extend/retract; the foot-position-IK/cascaded-LQI
// consumers that used to read it (posture_controler(), etc.) were removed as
// dead code under the NN-drive policy, which holds the legs at a fixed joint
// angle directly from main.c instead.
float base_target_y = -15.0f;
