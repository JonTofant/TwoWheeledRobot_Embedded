/*
 * joystic.h
 *
 *  Created on: Oct 22, 2025
 *      Author: jon
 */

#ifndef INC_JOYSTICK_H_
#define INC_JOYSTICK_H_

#include "stdint.h"

#define UART3_CONTROLLER_PACKET_LEN 11
extern float scale_speed;

// CONTROLLER VARIABLES
extern uint8_t uart3_controller_byte;
extern uint8_t uart3_controller_buf[UART3_CONTROLLER_PACKET_LEN];
extern uint8_t uart3_controller_index;
extern uint8_t uart3_controller_packet_ready;

extern int16_t axisLX;
extern uint16_t throttle;
extern uint16_t brake;
extern uint8_t xPressed;
extern uint8_t dpadUp;
extern uint8_t dpadDown;
extern uint8_t startPressed;

extern float desired_x_dualshock;
extern float desired_angle_dualshock; // Desired angle from the dualshock controller

#endif /* INC_JOYSTICK_H_ */
