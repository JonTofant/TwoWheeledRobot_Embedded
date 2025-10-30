/*
 * DDSM115.h
 *
 *  Created on: Mar 6, 2025
 *      Author: Jon Tofant
 */

#ifndef INC_DDSM115_H_
#define INC_DDSM115_H_

#include "stm32f4xx_hal.h"  // This includes the HAL definitions including HAL_StatusTypeDef.
#include <stdbool.h>
#include <stdint.h>
#include <math.h>

#define MAX_MOTORS_DDSM115 2

extern float motor_dt;

// Motor Parameters
extern const float MOTOR_TORQUE_CONSTANT_KT; // Nm/A (DDSM115)

// Defined motor variables for either RS485 comm protocol or the motor itself
#define RS485_BUFFER_SIZE 10

#define RAW_POS_MAX_COUNT 32768.0f // Max raw value + 1 (0 to 32767 = 32768 distinct values), use float for division
#define RAW_POS_HALF_RANGE (RAW_POS_MAX_COUNT / 2) // Threshold for wrap detection = 16384

extern const uint32_t ENCODER_FULL_RANGE_COUNTS;
extern const float ENCODER_HALF_RANGE_COUNTS;
extern const float COUNTS_TO_RADIANS_FACTOR_PHI;
extern const float RPM_TO_RAD_PER_SEC;

#define WHEEL_RADIUS_R 0.0505f

extern float DZ_RIGHT_POS;
extern float DZ_RIGHT_NEG;
extern float DZ_LEFT_POS;
extern float DZ_LEFT_NEG;


// Motor structure definition with an error flag.
typedef struct {
    uint8_t motorID;

    float target_angle;
    float target_current;
    float min_angle;
    float max_angle;
    bool  errorFlag;

    // State
    float phi_rad;
    float phi_dot_rad_s;
    float x;
    float x_dot;
    float x_ddot;

    float prev_x_dot;

    // Internal
    int32_t num_rotations;
    uint16_t prev_raw_pos;
    bool initialized;

    // NEW:
    float phi_zero;  // First reading used as zero reference
    bool  phi_zero_initialized;
    uint32_t last_update_time_us;
    float motor_dt;               // actual sample time in seconds

} DDSM115;



// Extern declaration of motor array.
extern DDSM115 DDSM115MotorList[MAX_MOTORS_DDSM115];


uint8_t compute_crc8(uint8_t *data, uint8_t len);
void sendPositionCommand(uint8_t motorID, float angle_deg);
uint16_t angleToValue(float angle_deg);
void DDMS115setMode(uint8_t motorID, uint8_t mode);
void DDSM115setCurrent(uint8_t motorID, float current_amp);
void DDSM115ChangeID(uint8_t motorID, uint8_t newID);
void update_ddsm115_state(DDSM115* motor, const uint8_t* Buffer, float wheel_radius);


#endif /* INC_DDSM115_H_ */
