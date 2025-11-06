/*
 * state_machine.c
 *
 *  Created on: Nov 6, 2025
 *      Author: jon
 */

#include "state_machine.h"

volatile bool isCANReady = false;
volatile bool isDDSM115Ready = false;
volatile bool isControllerReady = false;
volatile bool isCYBERGEARReady = false;
volatile bool isTELEMETRYReady = true;
volatile bool isFallen = true;
volatile bool isLOCOMOTION = false;
volatile bool isSTATIC = true;
volatile bool isJUMP = false;
volatile bool uartSynced = false;
volatile bool isDEMO = true;
