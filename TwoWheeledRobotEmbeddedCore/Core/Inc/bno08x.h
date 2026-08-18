/*
 * bno08x.h
 *
 * Direct I2C3 driver for the BNO08x IMU, wired straight to the STM32
 * (no ESP32 bridge). Glue layer on top of the vendored CEVA/Hillcrest SH2
 * reference stack (Middlewares/SH2Sensorhub) — ported from the working
 * demo at SDV-Robot-main/STM/MAIN (SH2 Sensorhub/demo_app.c).
 */

#ifndef INC_BNO08X_H_
#define INC_BNO08X_H_

#include <stdbool.h>
#include "main.h"

// BNO08x breakout ADDR pin low = 0x4A, high = 0x4B (7-bit, pre-shifted for
// STM32 HAL's 8-bit addressing convention).
#define BNO08X_I2C_ADDR  (0x4B << 1)

void BNO08x_Init(I2C_HandleTypeDef *hi2c, uint16_t intPin);
/* Returns true when an asserted INT caused one or more reports to be read. */
bool BNO08x_Service(void);
void BNO08x_EXTI_Callback(uint16_t GPIO_Pin);

// Body-frame gravity vector [m/s^2] (SH2_GRAVITY report). Feeds
// STM32_DEPLOYMENT.md's pitch = atan2(g_y,-g_z), roll = atan2(g_x,-g_z).
extern volatile float bno_gravity_x, bno_gravity_y, bno_gravity_z;

// Calibrated gyroscope [rad/s] (SH2_GYROSCOPE_CALIBRATED report).
extern volatile float bno_gx, bno_gy, bno_gz;

// Game rotation vector quaternion (gyro+accel fusion, no magnetometer —
// avoids interference from the nearby CyberGear CAN wiring/motors).
extern volatile float bno_qw, bno_qx, bno_qy, bno_qz;

// Sequence/timestamp of the most recently decoded game-rotation-vector report.
// The timestamp comes from SH-2 and is in microseconds.
extern volatile uint32_t bno_rotation_update_count;
extern volatile uint64_t bno_rotation_timestamp_us;

// Yaw [rad], derived from the quaternion above. Will drift slowly over
// long runs since there's no magnetometer correction — accepted tradeoff.
extern volatile float bno_yaw_rad;

// True once the first sensor report of each of the three enabled types has
// been received after init/reset.
extern volatile bool bno_data_valid;

#endif /* INC_BNO08X_H_ */
