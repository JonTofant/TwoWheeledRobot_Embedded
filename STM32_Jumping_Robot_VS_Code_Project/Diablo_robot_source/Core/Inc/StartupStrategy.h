/*
 * StrartupStrategy.h
 *
 *  Created on: Oct 22, 2025
 *      Author: jon
 */

#ifndef INC_STARTUPSTRATEGY_H_
#define INC_STARTUPSTRATEGY_H_

#include <stdint.h>
#include <stdbool.h>
#include "controler.h"
#include "StateEstimator.h"
#include "state_machine.h"


//Startup strategy
extern uint8_t attempt_for_amount_of_samples;
extern bool isStartupStrategy;
extern bool isStartupStategySuccess ;

// Function for startup strategy
void startup_strategy_control();

#endif /* INC_STARTUPSTRATEGY_H_ */
