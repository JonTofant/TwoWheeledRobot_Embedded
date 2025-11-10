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

extern float vel_err_L_telemetry;
extern float vel_err_R_telemetry;
extern float sliding_surface_L_telemetry;
extern float sliding_surface_R_telemetry;
extern float controler_output_sta_L_telemetry;
extern float controler_output_sta_R_telemetry;
extern float K_L_telemetry;
extern float K_R_telemetry;
extern float v_L_telemetry;
extern float v_R_telemetry;

// This structure defines the data payload for telemetry.
// All variables sent over UART should be included here.
typedef struct {
    // Motor Angles
    float motor_angle_rf;
    float motor_angle_rb;
    float motor_angle_lf;
    float motor_angle_lb;
    float roll_angle;

    // Controller Output (DDSM115 motor currents)
    float pd_output_right_motor;
    float pd_output_left_motor;

    // Cascade Controller Desired Angles
    float theta_des_l;
    float theta_des_r;

    // Desired Chassis Positions
    float xc_des_l;
    float xc_des_r;

    // Desired Wheel Positions
    float desired_x_left;
    float desired_x_right;
} TelemetryPayload_t;

// This structure defines the full packet, including SOF and checksum.
// The __attribute__((packed)) is important to prevent the compiler from adding padding bytes.
typedef struct __attribute__((packed)) {
    uint8_t sof;
    TelemetryPayload_t payload;
    uint8_t checksum;
} TelemetryPacket_t;


/**
  * @brief Populates and sends a telemetry packet over the specified UART using DMA.
  * @param huart Pointer to a UART_HandleTypeDef structure that contains
  *              the configuration information for the specified UART (e.g., &huart3).
  * @retval None
  */
void Send_Telemetry(UART_HandleTypeDef *huart);

/**
  * @brief Callback function to be called from the main HAL_UART_TxCpltCallback.
  * @brief This function signals that the UART DMA transfer is complete.
  * @param huart Pointer to a UART_HandleTypeDef structure.
  * @retval None
  */
void Telemetry_UART_TxCpltCallback(UART_HandleTypeDef *huart);


#endif /* INC_TELEMETRY_H_ */
