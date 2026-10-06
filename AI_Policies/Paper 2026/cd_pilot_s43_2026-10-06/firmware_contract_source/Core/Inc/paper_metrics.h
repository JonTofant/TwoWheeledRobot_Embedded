/*
 * paper_metrics.h
 *
 * Cycle-accurate timing and binary UART telemetry for the paper experiments.
 */

#ifndef INC_PAPER_METRICS_H_
#define INC_PAPER_METRICS_H_

#include "main.h"
#include <stdbool.h>
#include <stdint.h>

#define PAPER_METRICS_OBS_DIM       13u
#define PAPER_METRICS_ACTION_DIM     2u
#define PAPER_METRICS_WHEEL_COUNT    2u
#define PAPER_METRICS_LEG_COUNT      4u
#define PAPER_METRICS_DIAGNOSTIC_COUNT 4u
#define PAPER_METRICS_RUN_VALUE_COUNT 8u

/* Order is part of the UART protocol. Keep in sync with
 * Tools/paper_uart_capture.py and Docs/PAPER_TELEMETRY.md. */
typedef enum {
	PAPER_Q_IMU_READ = 0,
	PAPER_Q_ENCODER_READ,
	PAPER_Q_OBSERVATION,
	PAPER_Q_INFERENCE,
	PAPER_Q_ACTION,
	PAPER_Q_MOTOR_WRITE,
	PAPER_Q_TOTAL,
	PAPER_Q_CONTROL_PERIOD,
	PAPER_Q_RELEASE_LATENCY,
	PAPER_Q_RS485_CYCLE,
	PAPER_Q_IMU_AGE,
	PAPER_Q_WHEEL_AGE,
	PAPER_Q_LEG_AGE,
	PAPER_Q_COUNT
} PaperMetricQuantity;

typedef struct {
	uint32_t flags;
	float attitude[6];       /* pitch, pitch rate, roll, roll rate, yaw, yaw rate */
	float diagnostic[PAPER_METRICS_DIAGNOSTIC_COUNT];
	float position_m;
	float velocity_mps;
	float wheel_position_rad[PAPER_METRICS_WHEEL_COUNT];
	float wheel_velocity_radps[PAPER_METRICS_WHEEL_COUNT];
	float wheel_current_command_A[PAPER_METRICS_WHEEL_COUNT];
	float wheel_current_measured_A[PAPER_METRICS_WHEEL_COUNT];
	float leg_angle_rad[PAPER_METRICS_LEG_COUNT];
	float observation[PAPER_METRICS_OBS_DIM];
	float action[PAPER_METRICS_ACTION_DIM];
	/* theta, theta_init, theta_max_abs, theta_rms, current_rms,
	 * recovery_time, x_final, run_time */
	float run_value[PAPER_METRICS_RUN_VALUE_COUNT];
} PaperTelemetryState;

void PaperMetrics_Init(UART_HandleTypeDef *huart);
void PaperMetrics_Reset(void);
uint32_t PaperMetrics_CycleNow(void);
void PaperMetrics_RecordDuration(PaperMetricQuantity quantity, uint32_t start_cycle);
void PaperMetrics_RecordInterval(PaperMetricQuantity quantity,
                                 uint32_t start_cycle, uint32_t end_cycle);

void PaperMetrics_OnControlRelease(void);
uint32_t PaperMetrics_BeginControl(void);
void PaperMetrics_EndControl(uint32_t start_cycle);

void PaperMetrics_MarkImuUpdate(void);
void PaperMetrics_MarkWheelUpdate(uint8_t wheel_index);
void PaperMetrics_MarkWheelUpdateAt(uint8_t wheel_index, uint32_t update_cycle);
void PaperMetrics_MarkLegUpdate(uint8_t leg_index);
void PaperMetrics_UpdateObservationAges(void);

void PaperMetrics_BeginRs485Cycle(void);
void PaperMetrics_EndRs485Cycle(void);
void PaperMetrics_EndRs485CycleAt(uint32_t end_cycle);

bool PaperMetrics_Send(const PaperTelemetryState *state);
void PaperMetrics_UART_TxCpltCallback(UART_HandleTypeDef *huart);

#endif /* INC_PAPER_METRICS_H_ */
