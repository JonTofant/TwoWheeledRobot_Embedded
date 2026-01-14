/*
 * JumpStrategy.c
 *
 *  Created on: Jan 12, 2026
 *      Author: jon
 */


#include "JumpStrategy.h"

bool isJumpStrategy = false;

// Parameters
#include "controler.h"
#include "cybergear.h"

#define JUMP_EXTEND_Y   -24.0f
#define JUMP_RETRACT_Y  -19.0f
#define JUMP_HOLD_CYCLES 3
#define JUMP_FF_TORQUE 10.0f// number of main loop cycles to hold extended position

typedef enum {
    JUMP_IDLE = 0,
    JUMP_EXTENDED,
    JUMP_HOLD,
    JUMP_RETRACTING
} JumpState_t;

static JumpState_t jump_state = JUMP_IDLE;
static uint32_t jump_counter = 0;

void jump_strategy_control()
{
    switch(jump_state)
    {
        case JUMP_IDLE:
            // Start jump by setting maximum extension
            base_target_y = JUMP_EXTEND_Y;
            // Set the feed-forward torque for jump
            CyberGearMotorList[0].desired_torque_ff = JUMP_FF_TORQUE; // Left leg
            CyberGearMotorList[1].desired_torque_ff = -JUMP_FF_TORQUE; // Right leg
            CyberGearMotorList[2].desired_torque_ff = JUMP_FF_TORQUE; // Left leg
            CyberGearMotorList[3].desired_torque_ff = -JUMP_FF_TORQUE; // Right leg

            jump_state = JUMP_HOLD;
            jump_counter = 0;
            break;

        case JUMP_HOLD:
            // Hold legs extended for N cycles
            if(jump_counter < JUMP_HOLD_CYCLES) {
                jump_counter++;
            } else {
                jump_state = JUMP_RETRACTING;
            }
            break;

        case JUMP_RETRACTING:
            // Retract legs back to normal
            base_target_y = JUMP_RETRACT_Y;
            // Set the feed-forward torque for jump
            CyberGearMotorList[0].desired_torque_ff = 0.0f; // Left leg
            CyberGearMotorList[1].desired_torque_ff = 0.0f; // Right leg
            CyberGearMotorList[2].desired_torque_ff = 0.0f; // Left leg
            CyberGearMotorList[3].desired_torque_ff = 0.0f; // Right leg
            jump_state = JUMP_IDLE;
            isJumpStrategy = false;  // Jump finished
            break;

        case JUMP_EXTENDED:
        default:
            // Not used in this version
            break;
    }
}

