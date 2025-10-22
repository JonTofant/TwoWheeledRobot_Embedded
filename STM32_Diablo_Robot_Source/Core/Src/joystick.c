/*
 * joystick.c
 *
 *  Created on: Oct 22, 2025
 *      Author: jon
 */

#include "joystick.h"

// CONTROLLER VARIABLES
uint8_t uart3_controller_byte;
uint8_t uart3_controller_buf[UART3_CONTROLLER_PACKET_LEN];
uint8_t uart3_controller_index = 0;
uint8_t uart3_controller_packet_ready = 0;

int16_t axisLX;
uint16_t throttle;
uint16_t brake;
uint8_t xPressed;
uint8_t dpadUp;
uint8_t dpadDown;
uint8_t startPressed;

float desired_x_dualshock = 0.0f;
float desired_angle_dualshock = 0.0f; // Desired angle from the dualshock controller

float scale_speed = 35;
