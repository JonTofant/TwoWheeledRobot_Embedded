/*
 * telemetry.h
 *
 *  Created on: Jul 10, 2025
 *      Author: jon
 */

#ifndef INC_TELEMETRY_H_
#define INC_TELEMETRY_H_

#include "stm32f4xx_hal.h" // Required for UART_HandleTypeDef
#include <stdint.h>
#include <stdbool.h>
// Define the Start-of-Frame byte for the telemetry packet
#define TELEMETRY_SOF 0xAA

extern float theta_des_l_telemetry;
extern float theta_des_r_telemetry;

// (packed) -> Pack the payload to ensure floats are tightly grouped with no gaps
typedef struct __attribute__((packed)) {
// ALL OF THE DDSM115 STATES
	float DDSM115_motor_velocity_right;
	float DDSM115_motor_velocity_left;
	float DDSM115_current_right;
	float DDSM115_current_left;

	// CYBERGEAR MOTOR ANGLES
	float motor_angle_rf;
	float motor_angle_rb;
	float motor_angle_lf;
	float motor_angle_lb;
	float motor_desired_current_injection_rf;
	float motor_desired_current_injection_rb;
	float motor_desired_current_injection_lf;
	float motor_desired_current_injection_lb;
	float motor_Kp;
	float motor_Kd;


	// ESP32 ROLL ANGLE
	float roll_angle;

	// PD CONTROLLER OUTPUTS
	float pd_output_right_motor;
	float pd_output_left_motor;

	// DESIRED THETA VALUES
	float theta_des_r;
	float theta_des_l;

	// DESIRED XC VALUES
	float xc_des_r;
	float xc_des_l;

	// DESIRED VELOCITIES
	float desired_velocity_right;
	float desired_velocity_left;
} TelemetryPayload_t;

// This structure defines the full packet, including SOF and checksum.
// The __attribute__((packed)) is important to prevent the compiler from adding padding bytes.
typedef struct __attribute__((packed)) {
    uint8_t sof;        // Always 0xAA
    uint8_t len;        // Number of bytes in the payload (sizeof TelemetryPayload_t)
    TelemetryPayload_t payload;
    uint8_t checksum;   // Sum of payload bytes
} TelemetryPacket_t;



void Send_Telemetry(UART_HandleTypeDef *huart);

void Telemetry_UART_TxCpltCallback(UART_HandleTypeDef *huart);


#endif /* INC_TELEMETRY_H_ */
