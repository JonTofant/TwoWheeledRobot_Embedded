/*
 * JumpStrategy.h
 *
 *  Created on: Jan 12, 2026
 *      Author: jon
 */

#ifndef INC_JUMPSTRATEGY_H_
#define INC_JUMPSTRATEGY_H_


#include <stdint.h>
#include <stdbool.h>
#include "controler.h"
#include "StateEstimator.h"
#include "state_machine.h"

extern bool isJumpStrategy;


void jump_strategy_control();

#endif /* INC_JUMPSTRATEGY_H_ */
