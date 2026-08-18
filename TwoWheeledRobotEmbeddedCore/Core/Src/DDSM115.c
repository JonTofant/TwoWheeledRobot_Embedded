/*
 * DDSM115.c
 *
 *  Created on: Mar 6, 2025
 *      Author: tofant
 */
// Todo globalise this

#include "DDSM115.h"
#include <string.h>  // for memcpy
#include "main.h"
#include "math.h"
#include <stdint.h>

float dt = 0.02;

// Motor Parameters
const float MOTOR_TORQUE_CONSTANT_KT = 0.75f; // Nm/A (DDSM115)

const uint32_t ENCODER_FULL_RANGE_COUNTS = 32768;
const float ENCODER_HALF_RANGE_COUNTS = 16384.0f;
const float COUNTS_TO_RADIANS_FACTOR_PHI = (2.0f * M_PI) / (float)ENCODER_FULL_RANGE_COUNTS;
const float RPM_TO_RAD_PER_SEC = (2.0f * M_PI) / 60.0f; // ≈ 0.104719755f

/* Appendix B: 4.0679 counts/mA = 4067.9 counts/A. */
#define DDSM_CURRENT_FEEDBACK_COUNTS_PER_A 4067.9f


float DZ_RIGHT_POS = 0.04f;
float DZ_RIGHT_NEG = 0.04f;
float DZ_LEFT_POS  = 0.04f;
float DZ_LEFT_NEG  = 0.04f;

DDSM115 DDSM115MotorList[MAX_MOTORS_DDSM115] = {
	{ .motorID = 0x11, .target_angle = 0.0f, .min_angle = -6.283f, .max_angle = 6.283f, .errorFlag = true, .x =0, .x_dot =0, .x_ddot =0, .prev_x_dot = 0 },
	{ .motorID = 0x10, .target_angle = 0.0f, .min_angle = -6.283f, .max_angle = 6.283f, .errorFlag = true, .x = 0, .x_dot = 0, .x_ddot = 0, .prev_x_dot = 0}
};

volatile uint8_t DDSM115_last_rx[10];
volatile uint16_t DDSM115_last_rx_size;
volatile uint32_t DDSM115_rx_frame_count;
volatile uint8_t DDSM115_last_rx_crc_ok;
volatile uint8_t DDSM115_id_query_tx_status = (uint8_t)HAL_ERROR;


uint8_t position_mode[10] = {
    0x01,  // Motor ID
    0xA0,  // Command code for position loop command
    0x00,  // High byte of target position (10000 in decimal: 0x2710)
    0x00,  // Low byte of target position
    0x00,  // Reserved/unused (acceleration, brake, etc.)
    0x00,  // Reserved
    0x00,  // Reserved
    0x00,  // Reserved
    0x00,  // Reserved
    0x03,   // CRC8 checksum
};


uint8_t compute_crc8(uint8_t *data, uint8_t len) {
    uint8_t crc = 0;
    for (uint8_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (uint8_t j = 0; j < 8; j++) {
            if (crc & 0x01)
                crc = (crc >> 1) ^ 0x8C;
            else
                crc >>= 1;
        }
    }
    return crc;
}

void sendPositionCommand(uint8_t motorID, float angle_deg) {
    uint8_t command[10] = {0};
    uint16_t target_value = angleToValue(angle_deg);

    // Fill command packet according to documentation:
    // Byte 0: Motor ID, Byte 1: Command code (0x64 for drive command)
    command[0] = motorID;
    command[1] = 0x64; // Drive command code for position control

    // Bytes 2-3: 16-bit target position (big-endian)
    command[2] = (uint8_t)(target_value >> 8);   // High byte
    command[3] = (uint8_t)(target_value & 0xFF);   // Low byte

    // Bytes 4-8: Reserved (set to 0)
    command[4] = 0x00;
    command[5] = 0x00;
    command[6] = 0x00;
    command[7] = 0x00;
    command[8] = 0x00;

    // Byte 9: CRC8 checksum over bytes 0 to 8
    command[9] = compute_crc8(command, 9);

    // Set RS485 transceiver to transmit mode
    HAL_GPIO_WritePin(RS485_DIR_GPIO_Port, RS485_DIR_Pin, GPIO_PIN_SET);
    // Transmit the command
    HAL_UART_Transmit(&huart5, command, 10, HAL_MAX_DELAY);
    // Return RS485 transceiver to receive mode
    HAL_GPIO_WritePin(RS485_DIR_GPIO_Port, RS485_DIR_Pin, GPIO_PIN_RESET);
}


uint16_t angleToValue(float angle_deg) {
    // Clamp the angle between 0 and 360
    if(angle_deg < 0.0f) {
        angle_deg = 0.0f;
    }
    if(angle_deg > 360.0f) {
        angle_deg = 360.0f;
    }
    // Map 0-360° to 0-32767 (note: 32767 is the maximum unsigned 16-bit value used)
    return (uint16_t)((angle_deg / 360.0f) * 32767.0f);
}


// --- Set Mode Function ---
// mode should be one of:
//    0x01 -> current loop
//    0x02 -> velocity loop (if needed)
//    0x03 -> position loop
void DDMS115setMode(uint8_t motorID, uint8_t mode) {
    uint8_t cmd[10] = {0};
    cmd[0] = motorID;     // Motor ID
    cmd[1] = 0xA0;        // Command code for mode switching
    // Bytes 2-8 are reserved (set to 0)
    cmd[9] = mode;        // Mode value (no CRC calculation for mode command)

    // Set RS485 transceiver to transmit mode
    HAL_GPIO_WritePin(RS485_DIR_GPIO_Port, RS485_DIR_Pin, GPIO_PIN_SET);
    HAL_UART_Transmit(&huart5, cmd, 10, HAL_MAX_DELAY);
    HAL_Delay(10);  // Short delay to allow command processing
    // Return RS485 transceiver to receive mode
    HAL_GPIO_WritePin(RS485_DIR_GPIO_Port, RS485_DIR_Pin, GPIO_PIN_RESET);
}


// Broadcast ID query. Only one DDSM115 may be connected while this command is
// used; otherwise multiple replies collide on the shared RS485 bus.
HAL_StatusTypeDef DDSM115QueryID(void)
{
    uint8_t cmd[10] = {0};
    cmd[0] = 0xC8;
    cmd[1] = 0x64;
    cmd[9] = compute_crc8(cmd, 9);  // 0xDE

    DDSM115_last_rx_size = 0;
    DDSM115_rx_frame_count = 0;
    DDSM115_last_rx_crc_ok = 0;
    for (uint32_t i = 0; i < sizeof(DDSM115_last_rx); i++) {
        DDSM115_last_rx[i] = 0;
    }

    HAL_GPIO_WritePin(RS485_DIR_GPIO_Port, RS485_DIR_Pin, GPIO_PIN_SET);
    HAL_StatusTypeDef status = HAL_UART_Transmit(&huart5, cmd, sizeof(cmd),
                                                  HAL_MAX_DELAY);
    HAL_GPIO_WritePin(RS485_DIR_GPIO_Port, RS485_DIR_Pin, GPIO_PIN_RESET);
    DDSM115_id_query_tx_status = (uint8_t)status;
    return status;
}

void DDSM115CaptureRx(const uint8_t *buffer, uint16_t size)
{
    uint16_t copy_size = size;
    if (copy_size > sizeof(DDSM115_last_rx)) {
        copy_size = sizeof(DDSM115_last_rx);
    }

    for (uint16_t i = 0; i < copy_size; i++) {
        DDSM115_last_rx[i] = buffer[i];
    }
    DDSM115_last_rx_size = size;
    DDSM115_last_rx_crc_ok =
        (size == sizeof(DDSM115_last_rx) && compute_crc8((uint8_t *)buffer, 9) == buffer[9]) ? 1u : 0u;
    DDSM115_rx_frame_count++;
}


// --- Set Current Function ---
// In current loop mode, the motor expects a signed 16-bit value representing current,
// where -32767 corresponds to -8 A and +32767 corresponds to +8 A.
void DDSM115setCurrent(uint8_t motorID, float current_amp) {
    // Clamp current_amp to the range [-8.0, 8.0]
    if (current_amp > 8.0f) {
        current_amp = 8.0f;
    } else if (current_amp < -8.0f) {
        current_amp = -8.0f;
    }

    // Convert desired current to a signed 16-bit value:
    // current_value = (current_amp / 8.0) * 32767
    int16_t current_value = (int16_t)((current_amp / 8.0f) * 32767.0f);

    uint8_t cmd[10] = {0};
    cmd[0] = motorID;     // Motor ID
    cmd[1] = 0x64;        // Drive command code (used for sending current/position/speed)

    // Bytes 2-3: current_value as signed 16-bit (big-endian)
    cmd[2] = (uint8_t)((uint16_t)current_value >> 8);  // High byte
    cmd[3] = (uint8_t)(current_value & 0xFF);          // Low byte

    // Bytes 4-8: Reserved (set to 0)
    cmd[4] = 0x00;
    cmd[5] = 0x00;
    cmd[6] = 0x00;
    cmd[7] = 0x00;
    cmd[8] = 0x00;

    // Byte 9: CRC8 checksum calculated over bytes 0 through 8
    cmd[9] = compute_crc8(cmd, 9);

    // Set RS485 transceiver to transmit mode
    HAL_GPIO_WritePin(RS485_DIR_GPIO_Port, RS485_DIR_Pin, GPIO_PIN_SET);
    HAL_UART_Transmit(&huart5, cmd, 10, HAL_MAX_DELAY);
    // Return RS485 transceiver to receive mode
    HAL_GPIO_WritePin(RS485_DIR_GPIO_Port, RS485_DIR_Pin, GPIO_PIN_RESET);
}

// ---------------------------------------------------------------------------
// Non-blocking half-duplex request/response sequencer
// ---------------------------------------------------------------------------
// All DDSM115s share one RS485 (MAX485) line, and every drive command triggers
// a 10-byte reply frame from the addressed motor. Because the transceiver is
// half-duplex, we must NOT turn it back to transmit (to command the next motor)
// until the current motor's reply has fully arrived on the idle line - doing so
// drives DE over the incoming reply and that wheel's feedback is lost. The old
// fixed HAL_Delay() between the two commands was a blind guess at that turnaround.
//
// This sequencer instead advances to the next motor only when its predecessor's
// reply has actually been received (flagged from HAL_UARTEx_RxEventCallback via
// DDSM115_NotifyReply) or after a short safety timeout, without ever blocking the
// superloop on a fixed delay. Call DDSM115_QueueCurrents() once per control tick
// with the desired per-wheel currents, and DDSM115_Service() every superloop pass.
#define DDSM_REPLY_TIMEOUT_MS 3u   // fallback if a motor never answers (unplugged/off)

static uint8_t  ddsm_seq_id[MAX_MOTORS_DDSM115];
static float    ddsm_seq_current[MAX_MOTORS_DDSM115];
static uint8_t  ddsm_seq_len = 0;   // number of motors queued this cycle
static uint8_t  ddsm_seq_pos = 0;   // next motor to command; >= len means cycle done
static uint8_t  ddsm_seq_busy = 0;  // 1 while waiting for the current motor's reply
static uint8_t  ddsm_seq_wait_id = 0;
static uint32_t ddsm_seq_tx_tick = 0;

static volatile uint8_t ddsm_reply_id = 0;    // motorID of the most recent reply
static volatile uint8_t ddsm_reply_flag = 0;  // set by the RX ISR, cleared before each TX

// Load a fresh command cycle. Any unfinished previous cycle is abandoned (turnaround
// is a few ms, well inside the ~15ms control tick, so this normally never happens).
void DDSM115_QueueCurrents(uint8_t id0, float c0, uint8_t id1, float c1)
{
    ddsm_seq_id[0] = id0; ddsm_seq_current[0] = c0;
    ddsm_seq_id[1] = id1; ddsm_seq_current[1] = c1;
    ddsm_seq_len = 2;
    ddsm_seq_pos = 0;
    ddsm_seq_busy = 0;
}

// Called from HAL_UARTEx_RxEventCallback when a DDSM115 reply frame has been parsed.
void DDSM115_NotifyReply(uint8_t motorID)
{
    ddsm_reply_id = motorID;
    ddsm_reply_flag = 1;
}

// Advance the sequencer. Sends the next motor's command when the line is free
// (previous reply received, or timed out); otherwise returns immediately so the
// superloop keeps running during the bus turnaround.
void DDSM115_Service(void)
{
    if (ddsm_seq_pos >= ddsm_seq_len) {
        return;  // cycle complete / nothing queued
    }

    if (!ddsm_seq_busy) {
        // Line is free: send this motor's command and start waiting for its reply.
        ddsm_seq_wait_id = ddsm_seq_id[ddsm_seq_pos];
        ddsm_reply_flag = 0;  // discard any stale reply before we listen for this one
        DDSM115setCurrent(ddsm_seq_id[ddsm_seq_pos], ddsm_seq_current[ddsm_seq_pos]);
        ddsm_seq_tx_tick = HAL_GetTick();
        ddsm_seq_busy = 1;
    } else if ((ddsm_reply_flag && ddsm_reply_id == ddsm_seq_wait_id) ||
               ((HAL_GetTick() - ddsm_seq_tx_tick) >= DDSM_REPLY_TIMEOUT_MS)) {
        // Reply captured (feedback safe) or motor never answered: move to the next.
        ddsm_seq_busy = 0;
        ddsm_seq_pos++;
    }
}

// --- Change Motor ID Function ---
// This function changes the motor ID. The new ID should be unique and not conflict with other motors.

void DDSM115ChangeID(uint8_t motorID, uint8_t newID){
    uint8_t cmd[10] = {0};

    cmd[0] = 0xAA;     // Motor ID
    cmd[1] = 0x55;        // Drive command code (used for sending current/position/speed)

    // Bytes 2-3: current_value as signed 16-bit (big-endian)
    cmd[2] = 0x53;  // High byte
    cmd[3] = newID;          // Low byte

    // Bytes 4-8: Reserved (set to 0)
    cmd[4] = 0x00;
    cmd[5] = 0x00;
    cmd[6] = 0x00;
    cmd[7] = 0x00;
    cmd[8] = 0x00;

    // Byte 9: CRC8 checksum calculated over bytes 0 through 8
    cmd[9] = 0x00;

    // Sending 5 time in a for loop

    for (int i = 0; i < 5; i++) {
		// Set RS485 transceiver to transmit mode
		HAL_GPIO_WritePin(RS485_DIR_GPIO_Port, RS485_DIR_Pin, GPIO_PIN_SET);
		HAL_UART_Transmit(&huart5, cmd, 10, HAL_MAX_DELAY);
		// Return RS485 transceiver to receive mode
		HAL_GPIO_WritePin(RS485_DIR_GPIO_Port, RS485_DIR_Pin, GPIO_PIN_RESET);
		HAL_Delay(10);  // Allow time for the command to be processed

	}
}


void update_ddsm115_state(DDSM115* motor, const uint8_t* Buffer, float wheel_radius)
{
    if (Buffer[0] != motor->motorID) return;

    // 0. Current feedback (reply bytes 2-3, signed big-endian).
    motor->current_feedback_raw = (int16_t)(((uint16_t)Buffer[2] << 8) | Buffer[3]);
    motor->current_feedback_A = (float)motor->current_feedback_raw /
                                DDSM_CURRENT_FEEDBACK_COUNTS_PER_A;

    // 1. Velocity
    uint16_t raw_velocity = ((uint16_t)Buffer[4] << 8) | Buffer[5];
    int16_t rpm = (int16_t)raw_velocity;
    motor->phi_dot_rad_s = -(float)rpm * RPM_TO_RAD_PER_SEC;
    motor->x_dot = motor->phi_dot_rad_s * wheel_radius;

    // 2. Position
    uint16_t current_raw_pos = ((uint16_t)Buffer[6] << 8) | Buffer[7];

    if (!motor->initialized) {
        motor->prev_raw_pos = current_raw_pos;

        float raw_angle = (float)current_raw_pos * (2.0f * M_PI / RAW_POS_MAX_COUNT);
        motor->phi_zero = raw_angle;
        motor->phi_zero_initialized = true;

        motor->phi_rad = 0.0f;
        motor->num_rotations = 0;
        motor->x = 0.0f;
        motor->x_ddot = 0.0f; // Initialize acceleration
        motor->prev_x_dot = motor->x_dot; // Store initial velocity
        motor->initialized = true;
        return;
    }

    // 3. Acceleration
    if (dt > 0.0f) { // Avoid division by zero
        motor->x_ddot = -((motor->x_dot - motor->prev_x_dot) / dt);
    } else {
        motor->x_ddot = 0.0f; // Set acceleration to zero if dt is invalid
    }

    // 4. Wrap detection
    int32_t delta = (int32_t)current_raw_pos - (int32_t)motor->prev_raw_pos;
    if (delta > RAW_POS_HALF_RANGE) motor->num_rotations--;
    else if (delta < -RAW_POS_HALF_RANGE) motor->num_rotations++;

    float raw_angle = (float)current_raw_pos * (2.0f * M_PI / RAW_POS_MAX_COUNT);
    float unwrapped = raw_angle + motor->num_rotations * 2.0f * M_PI;
    motor->phi_rad = unwrapped - motor->phi_zero;
    motor->x = motor->phi_rad * wheel_radius;

    // Update previous velocity
    motor->prev_x_dot = motor->x_dot;
    motor->prev_raw_pos = current_raw_pos;
}
