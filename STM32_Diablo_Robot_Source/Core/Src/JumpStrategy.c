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

#define JUMP_EXTEND_Y   -27.0f
#define JUMP_RETRACT_Y  -15.0f
#define JUMP_HOLD_CYCLES 1   // number of main loop cycles to hold extended position

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
            jump_state = JUMP_IDLE;
            isJumpStrategy = false;  // Jump finished
            break;

        case JUMP_EXTENDED:
        default:
            // Not used in this version
            break;
    }
}

