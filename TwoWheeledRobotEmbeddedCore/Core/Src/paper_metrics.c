/*
 * paper_metrics.c
 *
 * The timing accumulator uses Welford's online algorithm. "Jitter" in the
 * exported stream is the sample standard deviation, in microseconds. WCET is
 * the maximum observed duration since the most recent reset.
 */

#include "paper_metrics.h"
#include <math.h>
#include <string.h>

#define PAPER_UART_MAGIC_0        0xA5u
#define PAPER_UART_MAGIC_1        0x5Au
#define PAPER_UART_VERSION        2u
#define PAPER_UART_SAMPLE_TYPE    1u

typedef struct {
	uint32_t count;
	float mean_us;
	float m2_us;
	float last_us;
	float max_us;
} PaperTimingAccumulator;

typedef struct __attribute__((packed)) {
	uint32_t sequence;
	uint32_t timestamp_us;
	uint32_t flags;
	uint32_t dropped_frames;
	float timing_last_us[PAPER_Q_COUNT];
	float timing_mean_us[PAPER_Q_COUNT];
	float timing_wcet_us[PAPER_Q_COUNT];
	float timing_jitter_us[PAPER_Q_COUNT];
	float attitude[6];
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
	float run_value[PAPER_METRICS_RUN_VALUE_COUNT];
} PaperTelemetryPayload;

typedef struct __attribute__((packed)) {
	uint8_t magic[2];
	uint8_t version;
	uint8_t type;
	uint16_t payload_size;
	PaperTelemetryPayload payload;
	uint16_t crc16;
} PaperTelemetryPacket;

typedef char PaperPayloadSizeMustRemain412Bytes[
	(sizeof(PaperTelemetryPayload) == 412u) ? 1 : -1];

static UART_HandleTypeDef *paper_uart;
static PaperTimingAccumulator timing[PAPER_Q_COUNT];
static uint32_t source_imu_cycle;
static uint32_t source_wheel_cycle[PAPER_METRICS_WHEEL_COUNT];
static uint32_t source_leg_cycle[PAPER_METRICS_LEG_COUNT];
static uint32_t previous_release_cycle;
static uint32_t pending_release_cycle;
static uint32_t rs485_start_cycle;
static float cycles_per_us = 1.0f;
static uint32_t sequence;
static uint32_t dropped_frames;
static volatile bool tx_busy;
static PaperTelemetryPacket tx_packet;

static uint16_t crc16_ccitt(const uint8_t *data, uint16_t length)
{
	uint16_t crc = 0xFFFFu;

	for (uint16_t i = 0; i < length; i++) {
		crc ^= (uint16_t)data[i] << 8;
		for (uint8_t bit = 0; bit < 8u; bit++) {
			crc = (crc & 0x8000u) ? (uint16_t)((crc << 1) ^ 0x1021u)
			                         : (uint16_t)(crc << 1);
		}
	}
	return crc;
}

static float cycles_to_us(uint32_t cycles)
{
	return (float)cycles / cycles_per_us;
}

static void record_us(PaperMetricQuantity quantity, float value_us)
{
	if ((uint32_t)quantity >= (uint32_t)PAPER_Q_COUNT) {
		return;
	}

	PaperTimingAccumulator *acc = &timing[quantity];
	acc->last_us = value_us;
	acc->count++;
	float delta = value_us - acc->mean_us;
	acc->mean_us += delta / (float)acc->count;
	acc->m2_us += delta * (value_us - acc->mean_us);
	if (value_us > acc->max_us) {
		acc->max_us = value_us;
	}
}

static bool all_sources_valid(const uint32_t *source, uint32_t count)
{
	for (uint32_t i = 0; i < count; i++) {
		if (source[i] == 0u) {
			return false;
		}
	}
	return true;
}

static uint32_t oldest_source_age(uint32_t now, const uint32_t *source, uint32_t count)
{
	uint32_t oldest_age = 0u;
	for (uint32_t i = 0; i < count; i++) {
		uint32_t age = now - source[i];
		if (age > oldest_age) {
			oldest_age = age;
		}
	}
	return oldest_age;
}

void PaperMetrics_Init(UART_HandleTypeDef *huart)
{
	paper_uart = huart;
	cycles_per_us = (float)SystemCoreClock / 1000000.0f;
	if (cycles_per_us < 1.0f) {
		cycles_per_us = 1.0f;
	}
	PaperMetrics_Reset();
}

void PaperMetrics_Reset(void)
{
	uint32_t primask = __get_PRIMASK();
	__disable_irq();
	memset(timing, 0, sizeof(timing));
	previous_release_cycle = 0u;
	pending_release_cycle = 0u;
	rs485_start_cycle = 0u;
	dropped_frames = 0u;
	if (!primask) {
		__enable_irq();
	}
}

uint32_t PaperMetrics_CycleNow(void)
{
	return DWT->CYCCNT;
}

void PaperMetrics_RecordDuration(PaperMetricQuantity quantity, uint32_t start_cycle)
{
	record_us(quantity, cycles_to_us(PaperMetrics_CycleNow() - start_cycle));
}

void PaperMetrics_OnControlRelease(void)
{
	uint32_t now = PaperMetrics_CycleNow();
	if (previous_release_cycle != 0u) {
		record_us(PAPER_Q_CONTROL_PERIOD, cycles_to_us(now - previous_release_cycle));
	}
	previous_release_cycle = now;
	pending_release_cycle = now;
}

uint32_t PaperMetrics_BeginControl(void)
{
	uint32_t now = PaperMetrics_CycleNow();
	if (pending_release_cycle != 0u) {
		record_us(PAPER_Q_RELEASE_LATENCY, cycles_to_us(now - pending_release_cycle));
		pending_release_cycle = 0u;
	}
	return now;
}

void PaperMetrics_EndControl(uint32_t start_cycle)
{
	PaperMetrics_RecordDuration(PAPER_Q_TOTAL, start_cycle);
}

void PaperMetrics_MarkImuUpdate(void)
{
	source_imu_cycle = PaperMetrics_CycleNow();
}

void PaperMetrics_MarkWheelUpdate(uint8_t wheel_index)
{
	if (wheel_index < PAPER_METRICS_WHEEL_COUNT) {
		source_wheel_cycle[wheel_index] = PaperMetrics_CycleNow();
	}
}

void PaperMetrics_MarkLegUpdate(uint8_t leg_index)
{
	if (leg_index < PAPER_METRICS_LEG_COUNT) {
		source_leg_cycle[leg_index] = PaperMetrics_CycleNow();
	}
}

void PaperMetrics_UpdateObservationAges(void)
{
	uint32_t now = PaperMetrics_CycleNow();
	if (source_imu_cycle != 0u) {
		record_us(PAPER_Q_IMU_AGE, cycles_to_us(now - source_imu_cycle));
	}
	if (all_sources_valid(source_wheel_cycle, PAPER_METRICS_WHEEL_COUNT)) {
		record_us(PAPER_Q_WHEEL_AGE,
		          cycles_to_us(oldest_source_age(now, source_wheel_cycle,
		                                         PAPER_METRICS_WHEEL_COUNT)));
	}
	if (all_sources_valid(source_leg_cycle, PAPER_METRICS_LEG_COUNT)) {
		record_us(PAPER_Q_LEG_AGE,
		          cycles_to_us(oldest_source_age(now, source_leg_cycle,
		                                         PAPER_METRICS_LEG_COUNT)));
	}
}

void PaperMetrics_BeginRs485Cycle(void)
{
	rs485_start_cycle = PaperMetrics_CycleNow();
}

void PaperMetrics_EndRs485Cycle(void)
{
	if (rs485_start_cycle != 0u) {
		PaperMetrics_RecordDuration(PAPER_Q_RS485_CYCLE, rs485_start_cycle);
		rs485_start_cycle = 0u;
	}
}

bool PaperMetrics_Send(const PaperTelemetryState *state)
{
	if (paper_uart == NULL || state == NULL || tx_busy) {
		if (tx_busy) {
			dropped_frames++;
		}
		return false;
	}

	tx_packet.magic[0] = PAPER_UART_MAGIC_0;
	tx_packet.magic[1] = PAPER_UART_MAGIC_1;
	tx_packet.version = PAPER_UART_VERSION;
	tx_packet.type = PAPER_UART_SAMPLE_TYPE;
	tx_packet.payload_size = (uint16_t)sizeof(PaperTelemetryPayload);
	tx_packet.payload.sequence = sequence++;
	tx_packet.payload.timestamp_us = HAL_GetTick() * 1000u;
	tx_packet.payload.flags = state->flags;
	tx_packet.payload.dropped_frames = dropped_frames;

	uint32_t primask = __get_PRIMASK();
	__disable_irq();
	for (uint32_t i = 0; i < PAPER_Q_COUNT; i++) {
		const PaperTimingAccumulator *acc = &timing[i];
		tx_packet.payload.timing_last_us[i] = acc->last_us;
		tx_packet.payload.timing_mean_us[i] = acc->mean_us;
		tx_packet.payload.timing_wcet_us[i] = acc->max_us;
		tx_packet.payload.timing_jitter_us[i] =
			(acc->count > 1u) ? sqrtf(acc->m2_us / (float)(acc->count - 1u)) : 0.0f;
	}
	if (!primask) {
		__enable_irq();
	}

	memcpy(tx_packet.payload.attitude, state->attitude, sizeof(state->attitude));
	memcpy(tx_packet.payload.diagnostic, state->diagnostic, sizeof(state->diagnostic));
	tx_packet.payload.position_m = state->position_m;
	tx_packet.payload.velocity_mps = state->velocity_mps;
	memcpy(tx_packet.payload.wheel_position_rad, state->wheel_position_rad,
	       sizeof(state->wheel_position_rad));
	memcpy(tx_packet.payload.wheel_velocity_radps, state->wheel_velocity_radps,
	       sizeof(state->wheel_velocity_radps));
	memcpy(tx_packet.payload.wheel_current_command_A, state->wheel_current_command_A,
	       sizeof(state->wheel_current_command_A));
	memcpy(tx_packet.payload.wheel_current_measured_A, state->wheel_current_measured_A,
	       sizeof(state->wheel_current_measured_A));
	memcpy(tx_packet.payload.leg_angle_rad, state->leg_angle_rad,
	       sizeof(state->leg_angle_rad));
	memcpy(tx_packet.payload.observation, state->observation, sizeof(state->observation));
	memcpy(tx_packet.payload.action, state->action, sizeof(state->action));
	memcpy(tx_packet.payload.run_value, state->run_value, sizeof(state->run_value));

	/* CRC covers version, type, payload_size and payload; the two magic bytes are
	 * intentionally excluded so a receiver can resynchronize on them. */
	tx_packet.crc16 = crc16_ccitt(&tx_packet.version,
	                              (uint16_t)(sizeof(tx_packet) - 4u));
	tx_busy = true;
	if (HAL_UART_Transmit_DMA(paper_uart, (uint8_t *)&tx_packet,
	                          (uint16_t)sizeof(tx_packet)) != HAL_OK) {
		tx_busy = false;
		dropped_frames++;
		return false;
	}
	return true;
}

void PaperMetrics_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
	if (paper_uart != NULL && huart->Instance == paper_uart->Instance) {
		tx_busy = false;
	}
}
