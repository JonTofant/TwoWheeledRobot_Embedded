/*
 * watchdog.c
 *
 *  Created on: Mar 12, 2025
 *      Author: tofan
 */

// Include the watchdog header file
#include "watchdog.h"
#include "cybergear.h"

// Define the watchdog function

void watchdog(CyberGear* motorList, DDSM115* motorListDDSM115, LegGeometry* legGeometryList, float dt){
	int error_count = 0;
	// Check if any angles in the motor list are out of bounds if any are stop all the motors
	for (int i = 0; i < MAX_MOTORS; i++){
		if (motorList[i].angle < motorList[i].min_angle || motorList[i].angle > motorList[i].max_angle){
			// Set the error flag to true
			motorList[i].errorFlag = true;
			error_count++;
		}
		else{
			// Set the error flag to false
			motorList[i].errorFlag = false;
		}
	}
	if (error_count > 0){
		for (int i = 0; i < MAX_MOTORS; i++){
			motorStop(&motorList[i]);
		}
	}
	else{
		error_count = 0;
	}
}

