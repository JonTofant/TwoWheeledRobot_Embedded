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

// Motor Parameters
extern const float MOTOR_TORQUE_CONSTANT_KT; // Nm/A (DDSM115)

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

    /* Raw reply field and calibrated engineering value. The scale is the
     * 12-unit below-deadzone fit reported in paper Appendix B. */
    int16_t current_feedback_raw;
    float current_feedback_A;

    float prev_x_dot;

    // Internal
    int32_t num_rotations;
    uint16_t prev_raw_pos;
    bool initialized;

    // NEW:
    float phi_zero;  // First reading used as zero reference
    bool  phi_zero_initialized;
} DDSM115;



// CyberGear Motor preprocessor macro
#define MOTOR_CG_LF CyberGearMotorList[0]
#define MOTOR_CG_LB CyberGearMotorList[1]
#define MOTOR_CG_RF CyberGearMotorList[2]
#define MOTOR_CG_RB CyberGearMotorList[3]

// Extern declaration of motor array.
extern DDSM115 DDSM115MotorList[MAX_MOTORS_DDSM115];

/* Raw RS485 diagnostics, captured before filtering on a configured motor ID. */
extern volatile uint8_t DDSM115_last_rx[10];
extern volatile uint16_t DDSM115_last_rx_size;
extern volatile uint32_t DDSM115_rx_frame_count;
extern volatile uint8_t DDSM115_last_rx_crc_ok;
extern volatile uint8_t DDSM115_startup_found_mask;
extern volatile uint8_t DDSM115_startup_ready;
extern volatile uint8_t DDSM115_startup_mode_before[MAX_MOTORS_DDSM115];
extern volatile uint8_t DDSM115_startup_mode_after[MAX_MOTORS_DDSM115];
extern volatile uint32_t DDSM115_transaction_count;
extern volatile uint32_t DDSM115_reply_count;
extern volatile uint32_t DDSM115_timeout_count;
extern volatile uint32_t DDSM115_bad_frame_count;


uint8_t compute_crc8(uint8_t *data, uint8_t len);
void sendPositionCommand(uint8_t motorID, float angle_deg);
uint16_t angleToValue(float angle_deg);
void DDMS115setMode(uint8_t motorID, uint8_t mode);
void DDSM115setCurrent(uint8_t motorID, float current_amp);
bool DDSM115InitializeCurrentMode(void);
bool DDSM115TransactCurrent(uint8_t motorID, float current_amp);
void DDSM115ChangeID(uint8_t motorID, uint8_t newID);
void update_ddsm115_state(DDSM115* motor, const uint8_t* Buffer, float wheel_radius);

#endif /* INC_DDSM115_H_ */
