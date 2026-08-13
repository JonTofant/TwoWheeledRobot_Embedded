/*
 * state_machine.h
 *
 *  Created on: Nov 6, 2025
 *      Author: jon
 */

#ifndef INC_STATE_MACHINE_H_
#define INC_STATE_MACHINE_H_

#include <stdbool.h>
#include "StateEstimator.h"

// OPERATIONAL STATES
extern volatile bool isCANReady;
extern volatile bool isDDSM115Ready;
extern volatile bool isControllerReady;
extern volatile bool isCYBERGEARReady;
extern volatile bool isLOCOMOTION;
extern volatile bool isSTATIC;
extern volatile bool isJUMP;
extern volatile bool isDEMO;

bool isFallen();


#endif /* INC_STATE_MACHINE_H_ */


