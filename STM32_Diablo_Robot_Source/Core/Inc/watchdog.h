/*
 * watchdog.h
 *
 *  Created on: Mar 12, 2025
 *      Author: tofan
 */

#ifndef INC_WATCHDOG_H_
#define INC_WATCHDOG_H_

#include "cybergear.h"
#include "DDSM115.h"
#include "kinematics.h"


// Define the watchdog function prototype, the input is the motor list
void watchdog(CyberGear* motorList, DDSM115* motorListDDSM115, LegGeometry* legGeometryList, float dt);


#endif /* INC_WATCHDOG_H_ */
