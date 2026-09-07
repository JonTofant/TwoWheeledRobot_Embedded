/*
 * bno08x.c
 *
 * Ported from SDV-Robot-main/STM/MAIN/SH2 Sensorhub/demo_app.c (working
 * bench-verified BNO08x-over-I2C driver on the same STM32F446 family) and
 * adapted to this project's pins (I2C3, BNO080_INT_Pin/BNO080_RST_Pin) and
 * to expose plain globals instead of a shared/overflowing float array.
 */

#include "bno08x.h"
#include "sh2.h"
#include "sh2_SensorValue.h"
#include "sh2_err.h"
#include <math.h>
#include <string.h>

static I2C_HandleTypeDef *_i2c;
static uint16_t           _intPin;
static sh2_Hal_t          _hal;
static volatile bool      _bnoIntFlag = false;

volatile float bno_gravity_x, bno_gravity_y, bno_gravity_z;
volatile float bno_gx, bno_gy, bno_gz;
volatile float bno_qw, bno_qx, bno_qy, bno_qz;
volatile uint32_t bno_rotation_update_count;
volatile uint64_t bno_rotation_timestamp_us;
volatile float bno_yaw_rad;
volatile bool  bno_data_valid = false;

static bool _haveGravity = false, _haveGyro = false, _haveRotation = false;

// ----------------------------------------------------------------
// Microsecond timing — reuses the DWT cycle counter main.c already
// enables globally at boot (CoreDebug_DEMCR_TRCENA_Msk / CYCCNTENA_Msk).
// ----------------------------------------------------------------
static uint32_t DWT_GetMicros(void)
{
    return (uint32_t)(DWT->CYCCNT / (SystemCoreClock / 1000000U));
}

// Hard ceiling on how long one BNO08x_Service() call may hold the caller.
//
// This matters because main.c's superloop is flag-driven and isCYBERGEARReady is
// a bool, not a counter: if a single pass overruns the 15 ms TIM4 period, the
// tick set during that pass is overwritten and silently lost. The stock value
// here was 10000 us, which against a 15 ms budget could eat the entire margin --
// the drain loop below runs while the INT pin stays asserted, and with three
// report streams the pin is almost never idle, so it ran to its ceiling nearly
// every pass. Measured effect on the 2026-09-07 5-degree run: the policy ticked
// at 49.3 Hz instead of 66.7 (one tick in four dropped) and the pitch it was fed
// refreshed at only 33.9 Hz.
//
// 2000 us keeps a pass well inside the tick even in the worst case. Anything the
// budget leaves undrained stays queued and is picked up on the next pass -- see
// the _bnoIntFlag handling at the end of BNO08x_Service(), which is what makes
// capping this safe with an edge-triggered INT.
#define BNO08X_SERVICE_BUDGET_US   4000U

// Busy-wait between the SHTP header read and the full-packet re-read in
// hal_read(). The BNO08x resets its output pointer on every I2C START, so the
// packet must be re-read from byte 0; this gap lets the sensor settle in
// between.
//
// THIS IS THE SINGLE BIGGEST COST IN THE DRIVER. It is spent per packet, spinning,
// with interrupts running but the superloop stalled. The ported value was 1000 us:
// at three report streams that is 300-600 packets/s, i.e. 4.5 ms of dead spin
// inside every 15 ms control tick, which is what was dropping ticks (66.7 Hz down
// to 49.3, then 58.4 after the budget cap above).
//
// 250 us is still an order of magnitude longer than the I2C byte time at 400 kHz.
// IF THE IMU DOES NOT COME UP after changing this -- bno_data_valid stays 0, which
// you can see in STM Studio within a second of boot, and LOG_theta_deg stays at
// exactly 0.000 -- put it back to 1000U. That is the whole revert.
#define BNO08X_READ_GAP_US         250U

// Per-stream report intervals. See BNO08x_Init for why they differ.
#define BNO08X_FAST_INTERVAL_US    10000U   // gravity + gyro: 100 Hz
#define BNO08X_YAW_INTERVAL_US     50000U   // rotation vector: 20 Hz

// ----------------------------------------------------------------
// SH2 HAL glue (I2C)
// ----------------------------------------------------------------
static int hal_open(sh2_Hal_t *self) { (void)self; return SH2_OK; }
static void hal_close(sh2_Hal_t *self) { (void)self; }

static int hal_read(sh2_Hal_t *self, uint8_t *buf, unsigned int len, uint32_t *t_us)
{
    (void)self;

    // Phase 1: read the 4-byte SHTP header to learn the packet length.
    if (HAL_I2C_Master_Receive(_i2c, BNO08X_I2C_ADDR, buf, 4, 100) != HAL_OK)
        return 0;

    uint16_t pktLen = ((uint16_t)(buf[1] & 0x7F) << 8) | buf[0];
    if (pktLen == 0 || pktLen < 4)
        return 0;
    if (pktLen > (uint16_t)len)
        pktLen = (uint16_t)len;

    uint32_t t_start = DWT_GetMicros();
    while (DWT_GetMicros() - t_start < BNO08X_READ_GAP_US)
        ;

    // Phase 2: re-read the entire packet from byte 0 — the BNO08x resets
    // its output pointer on every new I2C START condition.
    if (HAL_I2C_Master_Receive(_i2c, BNO08X_I2C_ADDR, buf, pktLen, 100) != HAL_OK)
        return 0;

    if (t_us)
        *t_us = DWT_GetMicros();
    return (int)pktLen;
}

static int hal_write(sh2_Hal_t *self, uint8_t *buf, unsigned int len)
{
    (void)self;
    if (HAL_I2C_Master_Transmit(_i2c, BNO08X_I2C_ADDR, buf, len, 100) != HAL_OK)
        return 0;
    return (int)len;
}

static uint32_t hal_getTimeUs(sh2_Hal_t *self)
{
    (void)self;
    return DWT_GetMicros();
}

// ----------------------------------------------------------------
// Sensor callback
// ----------------------------------------------------------------
static void sensorCallback(void *cookie, sh2_SensorEvent_t *event)
{
    (void)cookie;
    sh2_SensorValue_t val;

    if (sh2_decodeSensorEvent(&val, event) != SH2_OK)
        return;

    switch (val.sensorId)
    {
        case SH2_GRAVITY:
            bno_gravity_x = val.un.gravity.x;
            bno_gravity_y = val.un.gravity.y;
            bno_gravity_z = val.un.gravity.z;
            _haveGravity = true;
            break;

        case SH2_GYROSCOPE_CALIBRATED:
            bno_gx = val.un.gyroscope.x;
            bno_gy = val.un.gyroscope.y;
            bno_gz = val.un.gyroscope.z;
            _haveGyro = true;
            break;

        case SH2_GAME_ROTATION_VECTOR:
            bno_qw = val.un.gameRotationVector.real;
            bno_qx = val.un.gameRotationVector.i;
            bno_qy = val.un.gameRotationVector.j;
            bno_qz = val.un.gameRotationVector.k;

            {
                float siny_cosp = 2.0f * (bno_qw * bno_qz + bno_qx * bno_qy);
                float cosy_cosp = 1.0f - 2.0f * (bno_qy * bno_qy + bno_qz * bno_qz);
                bno_yaw_rad = atan2f(siny_cosp, cosy_cosp);
            }
            bno_rotation_timestamp_us = val.timestamp;
            bno_rotation_update_count++;
            _haveRotation = true;
            break;

        default:
            break;
    }

    if (_haveGravity && _haveGyro && _haveRotation)
        bno_data_valid = true;
}

// ----------------------------------------------------------------
// Event callback — re-enables reports after a BNO-side reset
// ----------------------------------------------------------------
static void eventCallback(void *cookie, sh2_AsyncEvent_t *event)
{
    (void)cookie;
    if (event->eventId == SH2_RESET)
    {
        bno_data_valid = false;
        _haveGravity = _haveGyro = _haveRotation = false;
        bno_rotation_update_count = 0u;
        bno_rotation_timestamp_us = 0u;

        // Must match BNO08x_Init's per-stream rates - see the rationale there.
        sh2_SensorConfig_t cfg = {0};
        cfg.reportInterval_us = BNO08X_FAST_INTERVAL_US;
        sh2_setSensorConfig(SH2_GYROSCOPE_CALIBRATED, &cfg);
        sh2_setSensorConfig(SH2_GRAVITY, &cfg);
        cfg.reportInterval_us = BNO08X_YAW_INTERVAL_US;
        sh2_setSensorConfig(SH2_GAME_ROTATION_VECTOR, &cfg);
    }
}

// ----------------------------------------------------------------
// Init
// ----------------------------------------------------------------
void BNO08x_Init(I2C_HandleTypeDef *hi2c, uint16_t intPin)
{
    _i2c    = hi2c;
    _intPin = intPin;
    bno_rotation_update_count = 0u;
    bno_rotation_timestamp_us = 0u;

    HAL_GPIO_WritePin(BNO080_RST_GPIO_Port, BNO080_RST_Pin, GPIO_PIN_RESET);
    HAL_Delay(10);
    HAL_GPIO_WritePin(BNO080_RST_GPIO_Port, BNO080_RST_Pin, GPIO_PIN_SET);
    HAL_Delay(300);

    _hal.open      = hal_open;
    _hal.close     = hal_close;
    _hal.read      = hal_read;
    _hal.write     = hal_write;
    _hal.getTimeUs = hal_getTimeUs;

    if (sh2_open(&_hal, eventCallback, NULL) != SH2_OK)
        return;

    sh2_setSensorCallback(sensorCallback, NULL);

    for (int i = 0; i < 20; i++) { sh2_service(); HAL_Delay(10); }

    // Report rates are NOT uniform, because every packet costs BNO08X_READ_GAP_US
    // of superloop time and the three streams are not equally urgent:
    //
    //   GRAVITY   -> theta. The balance axis. Must be fresh every control tick.
    //   GYRO      -> theta_dot, and the vector rotated into world Z for yaw rate.
    //                Same, it is the derivative term of the balance loop.
    //   ROTATION  -> yaw angle only, plus the quaternion used to rotate that gyro
    //                vector. Heading is a low-bandwidth term (the policy sees it
    //                as wrap_pi(yaw - yaw_ref)) and body orientation changes far
    //                slower than the gyro does, so a stale quaternion costs
    //                nothing measurable while a stale gravity vector costs a lot.
    //
    // 100 + 100 + 20 Hz instead of 3x100 cuts packet traffic by a quarter and
    // spends what is left on the two channels that actually hold the robot up.
    sh2_SensorConfig_t cfg = {0};
    cfg.reportInterval_us = BNO08X_FAST_INTERVAL_US;
    sh2_setSensorConfig(SH2_GYROSCOPE_CALIBRATED, &cfg);
    for (int i = 0; i < 10; i++) { sh2_service(); HAL_Delay(5); }
    sh2_setSensorConfig(SH2_GRAVITY, &cfg);
    for (int i = 0; i < 10; i++) { sh2_service(); HAL_Delay(5); }

    cfg.reportInterval_us = BNO08X_YAW_INTERVAL_US;
    sh2_setSensorConfig(SH2_GAME_ROTATION_VECTOR, &cfg);
    for (int i = 0; i < 10; i++) { sh2_service(); HAL_Delay(5); }
}

// ----------------------------------------------------------------
// App tick — call every main-loop iteration; no-ops unless the INT pin
// has signaled new data.
// ----------------------------------------------------------------
bool BNO08x_Service(void)
{
    if (!_bnoIntFlag)
        return false;

    uint32_t timeout = DWT_GetMicros();
    do {
        sh2_service();
    } while (HAL_GPIO_ReadPin(BNO080_INT_GPIO_Port, _intPin) == GPIO_PIN_RESET &&
             (DWT_GetMicros() - timeout) < BNO08X_SERVICE_BUDGET_US);

    // INT is configured GPIO_MODE_IT_FALLING, so no further interrupt arrives
    // while the pin stays asserted (low). If the budget expired with reports
    // still pending, the flag MUST stay set or the next pass would see
    // _bnoIntFlag == 0, return immediately, and the driver would wedge with the
    // pin low forever. Carrying it forward makes the cap purely a "finish this
    // on the next pass" split instead of a data loss.
    _bnoIntFlag = (HAL_GPIO_ReadPin(BNO080_INT_GPIO_Port, _intPin) == GPIO_PIN_RESET);

    HAL_NVIC_ClearPendingIRQ(EXTI1_IRQn);
    HAL_NVIC_EnableIRQ(EXTI1_IRQn);
    return true;
}

// ----------------------------------------------------------------
// EXTI callback — call from main.c's HAL_GPIO_EXTI_Callback when
// GPIO_Pin == BNO080_INT_Pin.
// ----------------------------------------------------------------
void BNO08x_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == _intPin &&
        HAL_GPIO_ReadPin(BNO080_INT_GPIO_Port, _intPin) == GPIO_PIN_RESET)
    {
        _bnoIntFlag = true;
        HAL_NVIC_DisableIRQ(EXTI1_IRQn);
    }
}
