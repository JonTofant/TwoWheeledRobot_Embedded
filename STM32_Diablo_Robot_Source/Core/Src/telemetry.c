/*
 * telemetry.c
 *
 *  Created on: Jul 10, 2025
 *      Author: jon
 */
#include "telemetry.h"
#include "main.h" // For HAL functions and access to global variables/structs
#include "cybergear.h"

/*
 * This file requires access to several global variables defined in main.c
 * or other modules. We declare them here as 'extern' to inform the compiler
 * that they exist elsewhere.
 */

// From main.c (or included headers like cybergear.h)
extern CyberGear CyberGearMotorList[MAX_MOTORS];
extern float roll_esp32;
extern float current_motor1_out;
extern float current_motor2_out;
extern float theta_des_l_telemetry;
extern float theta_des_r_telemetry;
extern float xc_des_l;
extern float xc_des_r;
extern float desired_v_left;
extern float desired_v_right;


float theta_des_l_telemetry;
float theta_des_r_telemetry;

// Motor macros from main.c for convenience
#define MOTOR_CG_LF CyberGearMotorList[0]
#define MOTOR_CG_LB CyberGearMotorList[1]
#define MOTOR_CG_RF CyberGearMotorList[2]
#define MOTOR_CG_RB CyberGearMotorList[3]

// Static variable to track DMA status. 'static' makes it private to this file.
static volatile bool telemetry_tx_ready = true;

/**
  * @brief Calculates a simple 8-bit checksum.
  * @param data Pointer to the data buffer.
  * @param len Length of the data.
  * @retval The calculated checksum.
  */
static uint8_t calculate_checksum(const uint8_t* data, uint16_t len) {
    uint8_t checksum = 0;
    for (uint16_t i = 0; i < len; i++) {
        checksum += data[i];
    }
    return checksum;
}

/**
  * @brief Populates and sends a telemetry packet over the specified UART using DMA.
  * @param huart Pointer to a UART_HandleTypeDef structure (e.g., &huart3).
  */
void Send_Telemetry(UART_HandleTypeDef *huart) {
    // 1. Check if the UART DMA is ready for a new transmission.
    if (!telemetry_tx_ready) {
        return; // Skip this cycle if the previous transmission is not yet complete.
    }

    // A static variable is used so this large struct is not allocated on the stack.
    static TelemetryPacket_t telemetry_packet;
    const uint16_t packet_size = sizeof(TelemetryPacket_t);

    // 2. Mark the transmitter as busy.
    telemetry_tx_ready = false;

    // 3. Fill the packet with data from your global variables.
    telemetry_packet.sof = TELEMETRY_SOF;

    // Motor Angles
    telemetry_packet.payload.motor_angle_rf = MOTOR_CG_RF.angle;
    telemetry_packet.payload.motor_angle_rb = MOTOR_CG_RB.angle;
    telemetry_packet.payload.motor_angle_lf = MOTOR_CG_LF.angle;
    telemetry_packet.payload.motor_angle_lb = MOTOR_CG_LB.angle;
    telemetry_packet.payload.roll_angle = roll_esp32;

    // Controller Output
    telemetry_packet.payload.pd_output_right_motor = current_motor1_out;
    telemetry_packet.payload.pd_output_left_motor  = current_motor2_out;

    // Desired angles from cascade controller
    telemetry_packet.payload.theta_des_l = theta_des_l_telemetry;
    telemetry_packet.payload.theta_des_r = theta_des_r_telemetry;

    // Desired chassis positions
    telemetry_packet.payload.xc_des_l = xc_des_l;
    telemetry_packet.payload.xc_des_r = xc_des_r;

    // Desired wheel positions
    telemetry_packet.payload.desired_x_left = desired_v_left;
    telemetry_packet.payload.desired_x_right = desired_v_right;

    // 4. Calculate the checksum over the payload part of the packet.
    telemetry_packet.checksum = calculate_checksum((uint8_t*)&telemetry_packet.payload, sizeof(TelemetryPayload_t));

    // 5. Start the DMA transmission.
    HAL_UART_Transmit_DMA(huart, (uint8_t*)&telemetry_packet, packet_size);
}

/**
  * @brief Callback to be called from the main HAL_UART_TxCpltCallback.
  */
void Telemetry_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    // Check if the callback is from the telemetry UART (USART3).
    // This makes the function safe if other UARTs also use Tx complete interrupts.
    if (huart->Instance == USART3) {
        // The transmission is complete, so we set the flag to true,
        // allowing the next call to Send_Telemetry() to proceed.
        telemetry_tx_ready = true;
    }
}

