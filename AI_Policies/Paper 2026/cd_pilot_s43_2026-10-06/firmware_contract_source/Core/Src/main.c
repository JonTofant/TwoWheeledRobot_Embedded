/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "app_x-cube-ai.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */


#include "math.h"
#include "string.h"
#include <stdbool.h>

// Custom headers
#include "cybergear.h"
#include "DDSM115.h"
#include "kinematics.h"
#include "system_init.h"
#include "controler.h"
#include "joystick.h"
#include "StartupStrategy.h"
#include "StateEstimator.h"
#include "state_machine.h"
#include "JumpStrategy.h"
#include "bno08x.h"
#include "paper_metrics.h"
#include "nn_arm_a_point.h"
#include "nn_arm_a_point_data.h"
#include "nn_arm_a_range.h"
#include "nn_arm_a_range_data.h"
#include "nn_arm_c_range_gru.h"
#include "nn_arm_c_range_gru_data.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

typedef enum {
	TESTBENCH_STAGE_IDLE = 0,
	TESTBENCH_STAGE_SETTLE,
	TESTBENCH_STAGE_LEFT_LEG,
	TESTBENCH_STAGE_NEUTRAL_AFTER_LEFT,
	TESTBENCH_STAGE_RIGHT_LEG,
	TESTBENCH_STAGE_NEUTRAL_AFTER_RIGHT,
	TESTBENCH_STAGE_CHECKPOINT_BEFORE_FORWARD,
	TESTBENCH_STAGE_FORWARD,
	TESTBENCH_STAGE_HOLD_AFTER_FORWARD,
	TESTBENCH_STAGE_CHECKPOINT_BEFORE_REVERSE,
	TESTBENCH_STAGE_REVERSE,
	TESTBENCH_STAGE_HOLD_AFTER_REVERSE,
	TESTBENCH_STAGE_SPIN_POSITIVE,
	TESTBENCH_STAGE_HOLD_AFTER_POSITIVE_SPIN,
	TESTBENCH_STAGE_SPIN_NEGATIVE,
	TESTBENCH_STAGE_HOLD_AFTER_NEGATIVE_SPIN,
	TESTBENCH_STAGE_CHECKPOINT_BEFORE_SINE_FORWARD,
	TESTBENCH_STAGE_SINE_LEGS_FORWARD,
	TESTBENCH_STAGE_CHECKPOINT_BEFORE_SINE_REVERSE,
	TESTBENCH_STAGE_SINE_LEGS_REVERSE,
	TESTBENCH_STAGE_FINAL_HOLD,
	TESTBENCH_STAGE_CROSSED_ASYMMETRIC_HOLD,
	TESTBENCH_STAGE_NEUTRAL_AFTER_CROSSED,
	TESTBENCH_STAGE_CHECKPOINT_BEFORE_ONE_LEG_FORWARD,
	TESTBENCH_STAGE_ONE_LEG_SINE_FORWARD,
	TESTBENCH_STAGE_CHECKPOINT_BEFORE_ONE_LEG_REVERSE,
	TESTBENCH_STAGE_ONE_LEG_SINE_REVERSE
} TestBenchStage;

typedef struct {
	float duration_s;
	float velocity_scale;
	float yaw_rate_scale;
	float left_leg_scale;
	float right_leg_scale;
	bool sine_legs;
} TestBenchStep;


/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define TESTBENCH_LEG_RAMP_S       1.0f
#define TESTBENCH_SINE_PERIOD_S    2.0f
#define TESTBENCH_DEG_TO_RAD       (3.14159265358979323846f / 180.0f)

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan1;

CRC_HandleTypeDef hcrc;

I2C_HandleTypeDef hi2c3;

SPI_HandleTypeDef hspi2;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart5;
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;
UART_HandleTypeDef huart6;
DMA_HandleTypeDef hdma_uart5_rx;
DMA_HandleTypeDef hdma_uart5_tx;
DMA_HandleTypeDef hdma_usart1_rx;
DMA_HandleTypeDef hdma_usart1_tx;
DMA_HandleTypeDef hdma_usart2_rx;
DMA_HandleTypeDef hdma_usart2_tx;
DMA_HandleTypeDef hdma_usart3_tx;

/* USER CODE BEGIN PV */

// Transcieve header
CAN_TxHeaderTypeDef pTxHeader;

// Receive header
CAN_RxHeaderTypeDef pRxHeader;

// Filter for receiving all extended frames
CAN_FilterTypeDef sFilterConfig;



uint8_t RxSingleByte;
uint16_t angle_u;
int16_t velocityDDSM_RPM = 0;

// ===== NN policy selection (compile-time, one active per build) =====
// Template-Twowheeledrobot-NNDriveFixedStance-v0 family (STM32_DEPLOYMENT.md): 13 obs -> 2
// actions (left/right DDSM115 wheel current, A). Legs are NOT policy-controlled - see
// NNDRIVE_LEG_NOMINAL_RAD below. Switching policies means rebuilding/reflashing.
#define POLICY_POINT_MLP  0   // nn_arm_a_point - point-training baseline, 13 in / 2 out
#define POLICY_RANGE_MLP  1   // nn_arm_a_range - range-randomized policy B, 13 in / 2 out
#define POLICY_RANGE_GRU  2   // nn_arm_c_range_gru - range-randomized recurrent policy C
#define ACTIVE_NN_POLICY  POLICY_RANGE_GRU

#define NNDRIVE_OBS_DIM    13
#define NNDRIVE_ACT_DIM    2
#define NNDRIVE_GRU_HIDDEN 64

#if ACTIVE_NN_POLICY == POLICY_POINT_MLP
#define NN_ACTIVATIONS_SIZE AI_NN_ARM_A_POINT_DATA_ACTIVATION_1_SIZE
#elif ACTIVE_NN_POLICY == POLICY_RANGE_MLP
#define NN_ACTIVATIONS_SIZE AI_NN_ARM_A_RANGE_DATA_ACTIVATION_1_SIZE
#elif ACTIVE_NN_POLICY == POLICY_RANGE_GRU
#define NN_ACTIVATIONS_SIZE AI_NN_ARM_C_RANGE_GRU_DATA_ACTIVATION_1_SIZE
#endif

static ai_handle nn_policy = AI_HANDLE_NULL;
static ai_buffer* nn_input  = NULL;
static ai_buffer* nn_output = NULL;
AI_ALIGNED(4) float nn_in_obs[NNDRIVE_OBS_DIM];      // not static: read by telemetry.c
AI_ALIGNED(4) float nn_out_action[NNDRIVE_ACT_DIM];  // not static: read by telemetry.c
#if ACTIVE_NN_POLICY == POLICY_RANGE_GRU
AI_ALIGNED(4) static float nn_gru_h[NNDRIVE_GRU_HIDDEN];      // persistent, threaded forward every tick
AI_ALIGNED(4) static float nn_gru_h_out[NNDRIVE_GRU_HIDDEN];  // copied into nn_gru_h after each run
#endif
AI_ALIGNED(32) static uint8_t nn_activations_pool[NN_ACTIVATIONS_SIZE];

// Legs are held at a constant pose, not policy-controlled (STM32_DEPLOYMENT.md
// "Legs (not policy-controlled)") - normal MIT position control, kp=30/kd=3 already the
// compiled-in default for every CyberGearMotorList[] entry (cybergear.c). Physical
// 10 degrees is the logical zero/nominal stance; test-bench motion is relative to it.
#define NNDRIVE_LEG_NOMINAL_DEG (10.0f)
#define NNDRIVE_LEG_NOMINAL_RAD (NNDRIVE_LEG_NOMINAL_DEG * TESTBENCH_DEG_TO_RAD)

// Debug/STM-Studio-tunable stand-ins for the joystick velocity/yaw-rate commands (no
// joystick wired up yet). Run through the exact same slew-limit + integration pipeline
// STM32_DEPLOYMENT.md specifies for v_joy/w_joy.
volatile float DBG_desired_velocity_mps  = 0.0f;
volatile float DBG_desired_yawrate_radps = 0.0f;

// Write DBG_testbench_start = 1 in Live Expressions to start. At a manual
// checkpoint, reposition and write DBG_testbench_continue = 1. Write start
// back to 0 at any time to abort. The amplitudes can be edited before each run.
volatile uint8_t DBG_testbench_start = 0u;
volatile uint8_t DBG_testbench_running = 0u;
volatile uint8_t DBG_testbench_waiting = 0u;
volatile uint8_t DBG_testbench_recovering = 0u;
volatile uint8_t DBG_testbench_continue = 0u;
volatile uint8_t DBG_testbench_fall_count = 0u;
volatile uint8_t DBG_testbench_stage = TESTBENCH_STAGE_IDLE;
volatile float DBG_testbench_stage_time_s = 0.0f;
volatile float DBG_testbench_linear_mps = 0.5f;
volatile float DBG_testbench_yaw_rate_radps = 1.0f;
volatile float DBG_testbench_leg_degrees = 35.0f;
static volatile uint8_t testbench_policy_reset_requested = 0u;

// Edit this table to change ordering or timing. Velocity/yaw/leg entries are
// multipliers for the three DBG_testbench_* amplitudes above.
// A negative duration marks a manual checkpoint. Set DBG_testbench_continue=1
// after repositioning to start the next movement segment.
static const TestBenchStep testbench_steps[] = {
	/* duration  velocity  yaw   left leg  right leg  sine */
	{ 3.0f,       0.0f,   0.0f,    0.0f,      0.0f,    false }, // settle
	{ 5.0f,       0.0f,   0.0f,    1.0f,      0.0f,    false }, // left leg extend/hold
	{ 2.0f,       0.0f,   0.0f,    0.0f,      0.0f,    false }, // neutral
	{ 5.0f,       0.0f,   0.0f,    0.0f,      1.0f,    false }, // right leg extend/hold
	{ 2.0f,       0.0f,   0.0f,    0.0f,      0.0f,    false }, // neutral
	{-1.0f,       0.0f,   0.0f,    0.0f,      0.0f,    false }, // checkpoint: reposition before forward
	{ 3.0f,      +1.0f,   0.0f,    0.0f,      0.0f,    false }, // forward
	{ 3.0f,       0.0f,   0.0f,    0.0f,      0.0f,    false }, // hold position
	{-1.0f,       0.0f,   0.0f,    0.0f,      0.0f,    false }, // checkpoint: reposition before reverse
	{ 3.0f,      -1.0f,   0.0f,    0.0f,      0.0f,    false }, // reverse
	{ 3.0f,       0.0f,   0.0f,    0.0f,      0.0f,    false }, // hold position
	{ 5.0f,       0.0f,  +1.0f,    0.0f,      0.0f,    false }, // positive yaw
	{ 3.0f,       0.0f,   0.0f,    0.0f,      0.0f,    false }, // hold heading
	{ 5.0f,       0.0f,  -1.0f,    0.0f,      0.0f,    false }, // negative yaw
	{ 3.0f,       0.0f,   0.0f,    0.0f,      0.0f,    false }, // hold heading
	{-1.0f,       0.0f,   0.0f,    0.0f,      0.0f,    false }, // checkpoint: symmetric-leg forward
	{ 4.0f,      +1.0f,   0.0f,    1.0f,      1.0f,    true  }, // two symmetric leg cycles forward
	{-1.0f,       0.0f,   0.0f,    0.0f,      0.0f,    false }, // checkpoint: symmetric-leg reverse
	{ 4.0f,      -1.0f,   0.0f,    1.0f,      1.0f,    true  }, // two symmetric leg cycles reverse
	{ 3.0f,       0.0f,   0.0f,    0.0f,      0.0f,    false }, // final hold
	{ 5.0f,       0.0f,   0.0f,    0.0f,      0.0f,    false }, // 1 s move + 4 s crossed asymmetric hold
	{ 1.0f,       0.0f,   0.0f,    0.0f,      0.0f,    false }, // smooth return from crossed pose
	{-1.0f,       0.0f,   0.0f,    0.0f,      0.0f,    false }, // checkpoint: one-leg forward
	{ 4.0f,      +1.0f,   0.0f,    1.0f,      0.0f,    false }, // two left-leg bump cycles forward
	{-1.0f,       0.0f,   0.0f,    0.0f,      0.0f,    false }, // checkpoint: one-leg reverse
	{ 4.0f,      -1.0f,   0.0f,    1.0f,      0.0f,    false }  // two left-leg bump cycles reverse
};

static float testbench_leg_target_rad[MAX_MOTORS] = {0.0f};
static float testbench_left_extension_deg = 0.0f;
static float testbench_right_extension_deg = 0.0f;
static float testbench_stage_start_left_deg = 0.0f;
static float testbench_stage_start_right_deg = 0.0f;

#define WHEEL_POS_SIGN_LEFT        (-1.0f)
#define WHEEL_POS_SIGN_RIGHT       (+1.0f)
#define WHEEL_VEL_SIGN_LEFT        (-1.0f)
#define WHEEL_VEL_SIGN_RIGHT       (+1.0f)

// Wheel-direction sign convention (STM32_DEPLOYMENT.md): kept as previously bench-verified
// for this robot's physical wiring - re-verify if the wiring ever changes.
#define DDSM_CURRENT_SIGN_LEFT     (+1.0f)
#define DDSM_CURRENT_SIGN_RIGHT    (-1.0f)
// Hardware yaw convention: applying logical [left,right] directly produced a
// measured positive-feedback yaw loop. Swapping only the physical destinations
// reverses the differential command while preserving the pitch/common command.
#define NNDRIVE_CURRENT_LIMIT_A (2.0f)      // Matches the exported policy action range (already scaled to amperes)

#define NNDRIVE_TICK_DT_S         (0.015f)  // 15ms TIM4 tick (matches MET_SAMPLE_DT_S)

// obs[9]/[10]: previous logical policy currents. The physical DDSM direction
// signs are an actuator/wiring mapping and must not be fed back into the policy.
static float prev_wheel_current_A[2] = {0.0f, 0.0f};

// ===== Bench metrics for STM Studio (metrics_per_run_template.csv columns) =====
// All "MET_" variables are updated once per 15ms control tick (see MET_Update()).
// From STM Studio: write MET_reset = 1 to zero the accumulators and start timing a
// new run; the firmware clears it back to 0 automatically on the next tick.
// Read the MET_ values off the STM Studio watch after the run/scenario finishes
// and copy them into the matching CSV columns (initial_theta_deg, max_abs_theta_deg,
// rms_theta_deg, rms_I_A, t_rec_s, x_final_m).
#define MET_RAD2DEG                (180.0f / 3.14159265358979323846f)
#define MET_SAMPLE_DT_S            (0.015f)  // 15ms TIM4 tick
#define MET_THETA_DISTURBED_DEG    (2.0f)    // |theta| above this counts as "disturbed"
#define MET_THETA_SETTLED_DEG      (1.5f)    // |theta| must stay below this to count as recovered
#define MET_SETTLE_HOLD_S          (0.3f)    // how long it must stay settled before t_rec_s freezes

volatile uint8_t MET_reset             = 0;     // write 1 to start a new run
volatile float   MET_theta_deg         = 0.0f;  // instantaneous pitch (deg), for live viewing
volatile float   MET_theta_init_deg    = 0.0f;  // -> initial_theta_deg
volatile float   MET_theta_max_abs_deg = 0.0f;  // -> max_abs_theta_deg
volatile float   MET_theta_rms_deg     = 0.0f;  // -> rms_theta_deg
volatile float   MET_I_rms_A           = 0.0f;  // -> rms_I_A
volatile float   MET_t_rec_s           = 0.0f;  // -> t_rec_s
volatile float   MET_x_final_m         = 0.0f;  // -> x_final_m (keeps updating; last value before you stop = final)
volatile float   MET_run_time_s        = 0.0f;  // elapsed time since MET_reset (context, not a CSV column)

// ===== Per-tick telemetry for STM Studio (timeseries_log_template.csv columns) =====
// All "LOG_" variables are updated once per 15ms control tick (see LOG_Update()).
// Start the STM Studio recording the instant you let go of the robot (or, for
// undisturbed_balance, whenever you want t=0 to be) - that recorded sample becomes
// t_s = 0, and LOG_theta_deg from that first sample is your initial_theta_deg.
// obs_0..obs_12 are the exact 13 floats fed to the network (nn_in_obs, normalized) -
// adjust LOG_Update() if your training env logged obs in a different order/units.
// action_L/R are the network's raw wheel-current output (pre sign-flip, pre safety clamp).
// I_L/R_cmd_A is the command actually sent to the DDSM115 (post sign-flip/clamp).
// I_L/R_meas_A uses reply bytes 2-3 and the Appendix-B 4067.9-count/A calibration.
volatile float   LOG_theta_deg      = 0.0f;  // instantaneous pitch (deg); first recorded sample -> initial_theta_deg
volatile float   LOG_obs[NNDRIVE_OBS_DIM] = {0};   // -> obs_0..obs_12
volatile float   LOG_action_L       = 0.0f;  // -> action_L (raw wheel-current network output)
volatile float   LOG_action_R       = 0.0f;  // -> action_R (raw wheel-current network output)
volatile float   LOG_I_L_cmd_A      = 0.0f;  // -> I_L_cmd_A
volatile float   LOG_I_R_cmd_A      = 0.0f;  // -> I_R_cmd_A
volatile float   LOG_I_L_meas_A     = 0.0f;  // -> I_L_meas_A
volatile float   LOG_I_R_meas_A     = 0.0f;  // -> I_R_meas_A
static bool      ddsm_metrics_cycle_active = false;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_CAN1_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_UART5_Init(void);
static void MX_SPI2_Init(void);
static void MX_I2C3_Init(void);
static void MX_USART6_UART_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM2_Init(void);
static void MX_CRC_Init(void);
/* USER CODE BEGIN PFP */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan);
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart);
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart);
volatile uint16_t rxReadIndex = 0;       // Your software read pointer






float ddsm_NN_current_command[2] = {0};

void ANN_Init(void);
void ANN_Run(void);
static void NN_ReadImu(float *pitch, float *pitch_rate, float *roll, float *roll_rate,
                        float *yaw, float *yaw_rate);
static float clamp_float(float value, float min_value, float max_value);
static float wrap_pi(float angle_rad);
static void MET_Update(void);
static void LOG_Update(void);
static void PAPER_SendTelemetry(void);
static void DDSM115_UpdateCompletedMetrics(void);
static void TestBench_Update(void);
static void TestBench_Stop(void);


/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static uint8_t paper_uart_rx_byte;
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  /*for (int i = 0; i < 10; i++) {
	  // Right after MX_GPIO_Init();
	  HAL_GPIO_TogglePin(GPIOA, LED_BUILTIN_Pin);
	  HAL_Delay(200);
	  HAL_GPIO_TogglePin(GPIOA, LED_BUILTIN_Pin);
  }*/
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_CAN1_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_UART5_Init();
  MX_SPI2_Init();
  MX_I2C3_Init();
  MX_USART6_UART_Init();
  MX_USART1_UART_Init();
  MX_USART3_UART_Init();
  MX_USART2_UART_Init();
  MX_TIM2_Init();
  MX_CRC_Init();
  MX_X_CUBE_AI_Init();
  /* USER CODE BEGIN 2 */

  ANN_Init();

  /*char boot[60];
  sprintf(boot, "BOOT OK - NN init done\r\n");
  HAL_UART_Transmit(&huart3, (uint8_t*)boot, strlen(boot), 100);
  HAL_Delay(100);*/

  // ------ PRECISE SAMPLE TIME MEASUREMENTS ------

  // Enable TRC (Trace control) - needed to use DWT
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;

  // Reset the cycle counter
  DWT->CYCCNT = 0;

  // Enable the cycle counter
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

  // Paper telemetry through the Nucleo ST-Link VCP: USART2 PA2/PA3,
  // 921600 baud, 8N1. Sending uses DMA after the measured control section.
  PaperMetrics_Init(&huart2);
  HAL_UART_Receive_IT(&huart2, &paper_uart_rx_byte, 1);

  //------------------------------------------------


  // CAN
  // 1) Configure the CAN filter to accept *all* extended frames
  CAN_FilterTypeDef sFilterConfig;
  sFilterConfig.FilterBank = 0;
  sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
  sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
  sFilterConfig.FilterIdHigh = 0x0000;
  sFilterConfig.FilterIdLow  = 0x0000;
  sFilterConfig.FilterMaskIdHigh = 0x0000;
  sFilterConfig.FilterMaskIdLow  = 0x0000;
  sFilterConfig.FilterFIFOAssignment = CAN_FILTER_FIFO0;
  sFilterConfig.FilterActivation = ENABLE;
  sFilterConfig.SlaveStartFilterBank = 14;
  HAL_CAN_ConfigFilter(&hcan1, &sFilterConfig);



  // 2) Start CAN
  HAL_CAN_Start(&hcan1);



  // 3) Optionally enable Rx interrupt
  HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);


  HAL_NVIC_SetPriority(CAN1_RX0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(CAN1_RX0_IRQn);





  	  // HAL delay to allow the system to stabilize
  	  HAL_Delay(3000);



  	  System_Init();

  // BNO08x IMU, wired directly to I2C3 (PA8/PC9) - see bno08x.h. Keep this
  // after the startup stabilization delay and System_Init(), matching the
  // known-working WalkingRobot startup order.
  BNO08x_Init(&hi2c3, BNO080_INT_Pin);

		  // Enable Timer 3 interupt
		  HAL_TIM_Base_Start_IT(&htim3);
		  // Enable Timer 4 interupt
		  HAL_TIM_Base_Start_IT(&htim4);

		  // 6) (Re-)enable the line just in case
		  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

		  HAL_UART_Receive_IT(&huart3, &uart3_controller_byte, 1);  // Start receiving




  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  while (1)
  {

	  // ----- MAIN CONTROL LOOP -----
	  // Seperated by flags for auxiliary tasks and a state machine

	 // UART5 DMA transactions advance from IRQ callbacks. This service only
	 // enforces the bounded no-reply deadline; it never waits for a motor.
	 DDSM115Service();
	 DDSM115_UpdateCompletedMetrics();

	 // Service the BNO08x (no-ops unless its INT pin has signaled new data) and
	 // refresh the derived pitch/roll/yaw state every pass, before anything below
	 // reads it (isFallen(), posture/startup/jump strategies, NN_ReadImu()).
	 uint32_t imu_read_start = PaperMetrics_CycleNow();
	 bool imu_updated = BNO08x_Service();
	 StateEstimator_UpdateFromBNO();
	 if (imu_updated) {
		 PaperMetrics_RecordDuration(PAPER_Q_IMU_READ, imu_read_start);
		 PaperMetrics_MarkImuUpdate();
	 }

	 if (isCANReady) {
		 isCANReady = false;
		 }

	 if (uart3_controller_packet_ready) {
		 process_joystick_input();
		 uart3_controller_packet_ready = 0;
	 }



	 // Leg targets come from the fixed-stance hold, sent from the isDDSM115Ready
	 // block below (same 15ms tick). This block just drains the flag and keeps the
	 // fall-handling / jump-strategy / startup-strategy triggers running every tick.
	 if (isCYBERGEARReady) {
		 if (isFallen()) {
			 if (startPressed == 1) isStartupStrategy = true;
		 } else {
			 if (xPressed == 1) isJumpStrategy = true;
		 }

		 if (isStartupStrategy) {
			 startup_strategy_control();
		 }

		 if (isFallen()) {
			 // Safety stop uses the same bounded DMA path as normal control.
			 if (DDSM115_startup_ready && DDSM115IsIdle()) {
				 (void)DDSM115QueueCurrent(0x10, 0.0f);
				 (void)DDSM115QueueCurrent(0x11, 0.0f);
			 }
		 }

		 if (isJumpStrategy) {
			 jump_strategy_control();
		 }

		 isCYBERGEARReady = false;
	 }


	 // Run the policy and both DDSM115 transactions on the 15ms TIM4 tick.
	 if (isDDSM115Ready) {
	     isDDSM115Ready = false;
	     /* Catch a reply or deadline reached while the IMU was being serviced,
	      * before observation ages and wheel state are consumed. */
	     DDSM115Service();
	     DDSM115_UpdateCompletedMetrics();
	     TestBench_Update();
	     uint32_t control_cycle_start = PaperMetrics_BeginControl();

	     PaperMetrics_UpdateObservationAges();
	     ANN_Run();
	     uint32_t motor_write_start = PaperMetrics_CycleNow();

	         if (!isFallen() || isStartupStrategy) {
	             // Legs are not policy-controlled - hold the fixed hardware-failsafe pose
	             // every tick (STM32_DEPLOYMENT.md "Legs (not policy-controlled)").
	             CyberGearMotorList[0].desired_angle = DBG_testbench_running ?
	                 testbench_leg_target_rad[0] : +NNDRIVE_LEG_NOMINAL_RAD;
	             CyberGearMotorList[1].desired_angle = DBG_testbench_running ?
	                 testbench_leg_target_rad[1] : -NNDRIVE_LEG_NOMINAL_RAD;
	             CyberGearMotorList[2].desired_angle = DBG_testbench_running ?
	                 testbench_leg_target_rad[2] : +NNDRIVE_LEG_NOMINAL_RAD;
	             CyberGearMotorList[3].desired_angle = DBG_testbench_running ?
	                 testbench_leg_target_rad[3] : -NNDRIVE_LEG_NOMINAL_RAD;

	             // No inter-command delay: Motor_SendMITCommand() waits for a free
	             // CAN TX mailbox internally, so all four frames queue back-to-back.
	             Motor_SendMITCommand(&CyberGearMotorList[0]);
	             Motor_SendMITCommand(&CyberGearMotorList[1]);
	             Motor_SendMITCommand(&CyberGearMotorList[2]);
	             Motor_SendMITCommand(&CyberGearMotorList[3]);
	         }
	         // else: fallen and not in startup strategy - don't send new MIT commands
	         // this tick, legs hold their last state per the CyberGear's own timeout.

	         if ((!isFallen() || isStartupStrategy) && DDSM115_startup_ready) {
		     if (DDSM115IsIdle()) {
		         PaperMetrics_BeginRs485Cycle();
		         ddsm_metrics_cycle_active = true;
		         (void)DDSM115QueueCurrent(0x11, ddsm_NN_current_command[0]);
		         (void)DDSM115QueueCurrent(0x10, ddsm_NN_current_command[1]);
		     }
		 }
	     PaperMetrics_RecordDuration(PAPER_Q_MOTOR_WRITE, motor_write_start);
	     PaperMetrics_EndControl(control_cycle_start);

	     // Bench metrics/telemetry for STM Studio (see MET_/LOG_ globals above), same 15ms tick.
	     MET_Update();
	     LOG_Update();
	     PAPER_SendTelemetry();
	 }
    /* USER CODE END WHILE */

  MX_X_CUBE_AI_Process();
    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief CAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN1_Init(void)
{

  /* USER CODE BEGIN CAN1_Init 0 */

  /* USER CODE END CAN1_Init 0 */

  /* USER CODE BEGIN CAN1_Init 1 */

  /* USER CODE END CAN1_Init 1 */
  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 2;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_12TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_8TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = DISABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = DISABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN1_Init 2 */

  /* USER CODE END CAN1_Init 2 */

}

/**
  * @brief CRC Initialization Function
  * @param None
  * @retval None
  */
static void MX_CRC_Init(void)
{

  /* USER CODE BEGIN CRC_Init 0 */

  /* USER CODE END CRC_Init 0 */

  /* USER CODE BEGIN CRC_Init 1 */

  /* USER CODE END CRC_Init 1 */
  hcrc.Instance = CRC;
  if (HAL_CRC_Init(&hcrc) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CRC_Init 2 */

  /* USER CODE END CRC_Init 2 */

}

/**
  * @brief I2C3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C3_Init(void)
{

  /* USER CODE BEGIN I2C3_Init 0 */

  /* USER CODE END I2C3_Init 0 */

  /* USER CODE BEGIN I2C3_Init 1 */

  /* USER CODE END I2C3_Init 1 */
  hi2c3.Instance = I2C3;
  hi2c3.Init.ClockSpeed = 400000;
  hi2c3.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c3.Init.OwnAddress1 = 0;
  hi2c3.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c3.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c3.Init.OwnAddress2 = 0;
  hi2c3.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c3.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C3_Init 2 */

  /* USER CODE END I2C3_Init 2 */

}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 84-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 100000-1;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 167;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 49999;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 20;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 59999;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

}

/**
  * @brief UART5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART5_Init(void)
{

  /* USER CODE BEGIN UART5_Init 0 */

  /* USER CODE END UART5_Init 0 */

  /* USER CODE BEGIN UART5_Init 1 */

  /* USER CODE END UART5_Init 1 */
  huart5.Instance = UART5;
  huart5.Init.BaudRate = 115200;
  huart5.Init.WordLength = UART_WORDLENGTH_8B;
  huart5.Init.StopBits = UART_STOPBITS_1;
  huart5.Init.Parity = UART_PARITY_NONE;
  huart5.Init.Mode = UART_MODE_TX_RX;
  huart5.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart5.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart5) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART5_Init 2 */

  /* USER CODE END UART5_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 3000000;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 921600;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief USART6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART6_UART_Init(void)
{

  /* USER CODE BEGIN USART6_Init 0 */

  /* USER CODE END USART6_Init 0 */

  /* USER CODE BEGIN USART6_Init 1 */

  /* USER CODE END USART6_Init 1 */
  huart6.Instance = USART6;
  huart6.Init.BaudRate = 3000000;
  huart6.Init.WordLength = UART_WORDLENGTH_8B;
  huart6.Init.StopBits = UART_STOPBITS_1;
  huart6.Init.Parity = UART_PARITY_NONE;
  huart6.Init.Mode = UART_MODE_TX_RX;
  huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart6.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart6) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART6_Init 2 */

  /* USER CODE END USART6_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream0_IRQn);
  /* DMA1_Stream3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream3_IRQn);
  /* DMA1_Stream5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);
  /* DMA1_Stream6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream6_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream6_IRQn);
  /* DMA1_Stream7_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream7_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream7_IRQn);
  /* DMA2_Stream2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);
  /* DMA2_Stream7_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream7_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream7_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, MRF_RESET_Pin|SPI2_CS_MRF_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(RS485_DIR_GPIO_Port, RS485_DIR_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(BNO080_RST_GPIO_Port, BNO080_RST_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GYRO_PS0_GPIO_Port, GYRO_PS0_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : MRF_RESET_Pin SPI2_CS_MRF_Pin GYRO_PS0_Pin */
  GPIO_InitStruct.Pin = MRF_RESET_Pin|SPI2_CS_MRF_Pin|GYRO_PS0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : BNO080_INT_Pin */
  GPIO_InitStruct.Pin = BNO080_INT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(BNO080_INT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : RS485_DIR_Pin */
  GPIO_InitStruct.Pin = RS485_DIR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(RS485_DIR_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : BNO080_RST_Pin */
  GPIO_InitStruct.Pin = BNO080_RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(BNO080_RST_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : MRF_INT_Pin */
  GPIO_InitStruct.Pin = MRF_INT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(MRF_INT_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

static void TestBench_Stop(void)
{
	DBG_testbench_start = 0u;
	DBG_testbench_running = 0u;
	DBG_testbench_waiting = 0u;
	DBG_testbench_recovering = 0u;
	DBG_testbench_continue = 0u;
	DBG_testbench_stage = TESTBENCH_STAGE_IDLE;
	DBG_testbench_stage_time_s = 0.0f;
	DBG_desired_velocity_mps = 0.0f;
	DBG_desired_yawrate_radps = 0.0f;
	testbench_left_extension_deg = 0.0f;
	testbench_right_extension_deg = 0.0f;
	testbench_stage_start_left_deg = 0.0f;
	testbench_stage_start_right_deg = 0.0f;
	testbench_leg_target_rad[0] = +NNDRIVE_LEG_NOMINAL_RAD;
	testbench_leg_target_rad[1] = -NNDRIVE_LEG_NOMINAL_RAD;
	testbench_leg_target_rad[2] = +NNDRIVE_LEG_NOMINAL_RAD;
	testbench_leg_target_rad[3] = -NNDRIVE_LEG_NOMINAL_RAD;
}

static void TestBench_Update(void)
{
	if (DBG_testbench_running == 0u) {
		if (DBG_testbench_start == 0u) {
			return;
		}
		if (isFallen() && !isStartupStrategy) {
			DBG_testbench_start = 0u;
			return;
		}

		DBG_testbench_running = 1u;
		DBG_testbench_waiting = 0u;
		DBG_testbench_recovering = 0u;
		DBG_testbench_continue = 0u;
		DBG_testbench_fall_count = 0u;
		DBG_testbench_stage = TESTBENCH_STAGE_SETTLE;
		DBG_testbench_stage_time_s = 0.0f;
		testbench_left_extension_deg = 0.0f;
		testbench_right_extension_deg = 0.0f;
		testbench_stage_start_left_deg = 0.0f;
		testbench_stage_start_right_deg = 0.0f;
		DBG_desired_velocity_mps = 0.0f;
		DBG_desired_yawrate_radps = 0.0f;
		testbench_policy_reset_requested = 1u;
		PaperMetrics_Reset();
		MET_reset = 1u;
	}

	// Setting the start switch back to zero is the emergency abort.
	if (DBG_testbench_start == 0u) {
		TestBench_Stop();
		return;
	}

	// A fall during an active stage marks that stage as failed and pauses with
	// neutral commands. After the robot is upright, continue advances to the next
	// stage instead of restarting the complete test.
	if (DBG_testbench_recovering != 0u) {
		DBG_desired_velocity_mps = 0.0f;
		DBG_desired_yawrate_radps = 0.0f;
		testbench_left_extension_deg = 0.0f;
		testbench_right_extension_deg = 0.0f;
		testbench_stage_start_left_deg = 0.0f;
		testbench_stage_start_right_deg = 0.0f;
		testbench_leg_target_rad[0] = +NNDRIVE_LEG_NOMINAL_RAD;
		testbench_leg_target_rad[1] = -NNDRIVE_LEG_NOMINAL_RAD;
		testbench_leg_target_rad[2] = +NNDRIVE_LEG_NOMINAL_RAD;
		testbench_leg_target_rad[3] = -NNDRIVE_LEG_NOMINAL_RAD;
		if (DBG_testbench_continue == 0u || isFallen()) {
			return;
		}

		DBG_testbench_continue = 0u;
		DBG_testbench_waiting = 0u;
		DBG_testbench_recovering = 0u;
		testbench_policy_reset_requested = 1u;
		DBG_testbench_stage++;
		// The crossed-pose return stage is unnecessary after recovery already
		// returned every leg to nominal.
		if (DBG_testbench_stage == TESTBENCH_STAGE_NEUTRAL_AFTER_CROSSED) {
			DBG_testbench_stage++;
		}
		DBG_testbench_stage_time_s = 0.0f;
		if ((uint32_t)DBG_testbench_stage >
		    (sizeof(testbench_steps) / sizeof(testbench_steps[0]))) {
			TestBench_Stop();
		}
		return;
	}

	if (DBG_testbench_waiting == 0u && isFallen() && !isStartupStrategy) {
		DBG_testbench_waiting = 1u;
		DBG_testbench_recovering = 1u;
		DBG_testbench_continue = 0u;
		if (DBG_testbench_fall_count < UINT8_MAX) {
			DBG_testbench_fall_count++;
		}
		testbench_policy_reset_requested = 1u;
		DBG_desired_velocity_mps = 0.0f;
		DBG_desired_yawrate_radps = 0.0f;
		testbench_left_extension_deg = 0.0f;
		testbench_right_extension_deg = 0.0f;
		testbench_stage_start_left_deg = 0.0f;
		testbench_stage_start_right_deg = 0.0f;
		testbench_leg_target_rad[0] = +NNDRIVE_LEG_NOMINAL_RAD;
		testbench_leg_target_rad[1] = -NNDRIVE_LEG_NOMINAL_RAD;
		testbench_leg_target_rad[2] = +NNDRIVE_LEG_NOMINAL_RAD;
		testbench_leg_target_rad[3] = -NNDRIVE_LEG_NOMINAL_RAD;
		return;
	}

	uint32_t step_index = (uint32_t)DBG_testbench_stage - 1u;
	if (step_index >= (sizeof(testbench_steps) / sizeof(testbench_steps[0]))) {
		TestBench_Stop();
		return;
	}

	const TestBenchStep *step = &testbench_steps[step_index];
	while (true) {
		if (step->duration_s < 0.0f) {
			if (DBG_testbench_waiting == 0u) {
				testbench_policy_reset_requested = 1u;
			}
			DBG_testbench_waiting = 1u;
			DBG_desired_velocity_mps = 0.0f;
			DBG_desired_yawrate_radps = 0.0f;
			testbench_left_extension_deg = 0.0f;
			testbench_right_extension_deg = 0.0f;
			testbench_stage_start_left_deg = 0.0f;
			testbench_stage_start_right_deg = 0.0f;
			testbench_leg_target_rad[0] = +NNDRIVE_LEG_NOMINAL_RAD;
			testbench_leg_target_rad[1] = -NNDRIVE_LEG_NOMINAL_RAD;
			testbench_leg_target_rad[2] = +NNDRIVE_LEG_NOMINAL_RAD;
			testbench_leg_target_rad[3] = -NNDRIVE_LEG_NOMINAL_RAD;
			if (DBG_testbench_continue == 0u || isFallen()) {
				return;
			}

			DBG_testbench_continue = 0u;
			DBG_testbench_waiting = 0u;
			testbench_policy_reset_requested = 1u;
			DBG_testbench_stage++;
			DBG_testbench_stage_time_s = 0.0f;
			step_index++;
		} else if (DBG_testbench_stage_time_s >= step->duration_s) {
			testbench_stage_start_left_deg = testbench_left_extension_deg;
			testbench_stage_start_right_deg = testbench_right_extension_deg;
			DBG_testbench_stage++;
			DBG_testbench_stage_time_s = 0.0f;
			step_index++;
		} else {
			break;
		}

		if (step_index >= (sizeof(testbench_steps) / sizeof(testbench_steps[0]))) {
			TestBench_Stop();
			return;
		}
		step = &testbench_steps[step_index];
	}

	DBG_desired_velocity_mps =
		step->velocity_scale * DBG_testbench_linear_mps;
	DBG_desired_yawrate_radps =
		step->yaw_rate_scale * DBG_testbench_yaw_rate_radps;

	if (DBG_testbench_stage == TESTBENCH_STAGE_ONE_LEG_SINE_FORWARD ||
	    DBG_testbench_stage == TESTBENCH_STAGE_ONE_LEG_SINE_REVERSE) {
		float phase = (2.0f * 3.14159265358979323846f *
		               DBG_testbench_stage_time_s) / TESTBENCH_SINE_PERIOD_S;
		testbench_left_extension_deg = DBG_testbench_leg_degrees *
		                               0.5f * (1.0f - cosf(phase));
		testbench_right_extension_deg = 0.0f;
	} else if (step->sine_legs) {
		float phase = (2.0f * 3.14159265358979323846f *
		               DBG_testbench_stage_time_s) / TESTBENCH_SINE_PERIOD_S;
		float extension_deg = DBG_testbench_leg_degrees *
		                      0.5f * (1.0f - cosf(phase));
		testbench_left_extension_deg = extension_deg;
		testbench_right_extension_deg = extension_deg;
	} else {
		float blend = clamp_float(DBG_testbench_stage_time_s /
		                          TESTBENCH_LEG_RAMP_S, 0.0f, 1.0f);
		// Raised-cosine interpolation avoids a step in leg angle or velocity.
		blend = 0.5f * (1.0f - cosf(3.14159265358979323846f * blend));
		float left_target_deg =
			step->left_leg_scale * DBG_testbench_leg_degrees;
		float right_target_deg =
			step->right_leg_scale * DBG_testbench_leg_degrees;
		testbench_left_extension_deg = testbench_stage_start_left_deg +
			(left_target_deg - testbench_stage_start_left_deg) * blend;
		testbench_right_extension_deg = testbench_stage_start_right_deg +
			(right_target_deg - testbench_stage_start_right_deg) * blend;
	}

	float left_rad = NNDRIVE_LEG_NOMINAL_RAD +
	                 testbench_left_extension_deg * TESTBENCH_DEG_TO_RAD;
	float right_rad = NNDRIVE_LEG_NOMINAL_RAD +
	                  testbench_right_extension_deg * TESTBENCH_DEG_TO_RAD;
	// Each physical leg uses two opposed CyberGears. These signs match the
	// existing folded-zero convention in system_init.c.
	testbench_leg_target_rad[0] = +left_rad;
	testbench_leg_target_rad[1] = -left_rad;
	testbench_leg_target_rad[2] = +right_rad;
	testbench_leg_target_rad[3] = -right_rad;

	if (DBG_testbench_stage == TESTBENCH_STAGE_CROSSED_ASYMMETRIC_HOLD) {
		float blend = clamp_float(DBG_testbench_stage_time_s /
		                          TESTBENCH_LEG_RAMP_S, 0.0f, 1.0f);
		blend = 0.5f * (1.0f - cosf(3.14159265358979323846f * blend));
		float crossed_rad = NNDRIVE_LEG_NOMINAL_RAD +
		                    DBG_testbench_leg_degrees * TESTBENCH_DEG_TO_RAD * blend;
		// Physical test showed indices 1 and 2 moving the same (front) pair,
		// so select the other left actuator to produce a crossed diagonal pose.
		testbench_leg_target_rad[0] = +crossed_rad; // left-back (physical)
		testbench_leg_target_rad[1] = -NNDRIVE_LEG_NOMINAL_RAD; // left-front (physical)
		testbench_leg_target_rad[2] = +crossed_rad; // right-front
		testbench_leg_target_rad[3] = -NNDRIVE_LEG_NOMINAL_RAD; // right-back
	} else if (DBG_testbench_stage == TESTBENCH_STAGE_NEUTRAL_AFTER_CROSSED) {
		float blend = clamp_float(DBG_testbench_stage_time_s /
		                          TESTBENCH_LEG_RAMP_S, 0.0f, 1.0f);
		blend = 0.5f * (1.0f - cosf(3.14159265358979323846f * blend));
		float crossed_rad = NNDRIVE_LEG_NOMINAL_RAD +
		                    DBG_testbench_leg_degrees * TESTBENCH_DEG_TO_RAD *
		                    (1.0f - blend);
		testbench_leg_target_rad[0] = +crossed_rad;
		testbench_leg_target_rad[1] = -NNDRIVE_LEG_NOMINAL_RAD;
		testbench_leg_target_rad[2] = +crossed_rad;
		testbench_leg_target_rad[3] = -NNDRIVE_LEG_NOMINAL_RAD;
	}

	DBG_testbench_stage_time_s += NNDRIVE_TICK_DT_S;
}

static void DDSM115_UpdateCompletedMetrics(void)
{
	uint8_t completed = DDSM115TakeCompletedMask();
	for (uint8_t motor_index = 0u; motor_index < MAX_MOTORS_DDSM115; motor_index++) {
		if ((completed & (uint8_t)(1u << motor_index)) != 0u) {
			uint32_t transaction_start =
				DDSM115GetCompletionStartCycle(motor_index);
			uint32_t transaction_end =
				DDSM115GetCompletionEndCycle(motor_index);
			PaperMetrics_RecordInterval(PAPER_Q_ENCODER_READ,
			                            transaction_start, transaction_end);
			PaperMetrics_MarkWheelUpdateAt(motor_index, transaction_end);
		}
	}

	if (ddsm_metrics_cycle_active && DDSM115IsIdle()) {
		PaperMetrics_EndRs485CycleAt(DDSM115GetLastTransactionEndCycle());
		ddsm_metrics_cycle_active = false;
	}
}


void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    HAL_StatusTypeDef status;
    status = HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &pRxHeader, CAN_received_data);
    if (status != HAL_OK)
    {
        // Error reading the message
        return;
    }

    uint32_t extId = pRxHeader.ExtId;
    uint8_t type   = (extId >> 24) & 0x1F;   // bits28..24
    uint8_t motorID= (extId >> 8) & 0xFFFF;  // Correct: Extracts bits 23–8
				   // bits7..0




    // Check what type we received
    if (type == 2) {  // Feedback message from motor


           // Extract current angle from Bytes 0-1.
           // Feedback spans -4pi..+4pi and MUST stay in the same frame Motor_SendMITCommand
           // encodes desired_angle in (0 = mechanical zero set at the folded stop): any
           // offset added here shifts every leg's desired_angle target away from true zero.
           uint16_t angle_raw = (CAN_received_data[0] << 8) | CAN_received_data[1];
           float angle = ((float)angle_raw / 65535.0f) * (8.0f * M_PI) - 4.0f * M_PI;

           // Extract angular velocity from Bytes 2-3 (raw 0 = -30 rad/s, raw 65535 = +30 rad/s)
           uint16_t velocity_raw = (CAN_received_data[2] << 8) | CAN_received_data[3];
           float velocity = ((float)velocity_raw / 65535.0f) * 60.0f - 30.0f;

           // Extract torque from Bytes 4-5 (raw 0 = -12 Nm, raw 65535 = +12 Nm)
           uint16_t torque_raw = (CAN_received_data[4] << 8) | CAN_received_data[5];
           float torque = ((float)torque_raw / 65535.0f) * 24.0f - 12.0f;

           // Extract temperature from Bytes 6-7
           uint16_t temp_raw = (CAN_received_data[6] << 8) | CAN_received_data[7];
           float temperature = (float)temp_raw / 10.0f; // Temp in Celsius

           // Find matching motor in motorList and update its values
           for (int i = 0; i < MAX_MOTORS; i++) {
               if (CyberGearMotorList[i].motorID == motorID) {
            	   CyberGearMotorList[i].angle = angle;
            	   CyberGearMotorList[i].velocity = velocity;
            	   CyberGearMotorList[i].torque = torque;
            	   CyberGearMotorList[i].temperature = temperature;
            	   //CyberGearMotorList[i].errorFlag = false;
				   CyberGearMotorList[i].update_flag = true;
				   PaperMetrics_MarkLegUpdate((uint8_t)i);


                   break;  // No need to check further once a match is found
               }
           }
           // Call watchdog function
           //watchdog(&CyberGearMotorList, &DDSM115MotorList, &LegGeometryList, &sampleTime);


       }
    else if (type == 17){




    }
    else if (type == 21)
    {
        // This is typically the fault/error frame
        // Byte0..3 might be a fault code


        // Optionally parse the fault code in received_data[0..3]
        // For example:
        /*uint32_t fault = (CAN_received_data[0])
                       | (CAN_received_data[1] << 8)
                       | (CAN_received_data[2] << 16)
                       | (CAN_received_data[3] << 24);*/
    }
    else if (type == 0)
    {
        // Possibly a "Get Device ID" response
        // The data bytes may contain the 64-bit unique ID
    }
    else
    {
        // Some other type

    }
}





void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if (huart->Instance == UART5) {
		DDSM115_UART_RxCpltCallback(huart);
		return;
	}
	if (huart->Instance == USART2) {
		if (paper_uart_rx_byte == 'R' || paper_uart_rx_byte == 'r') {
			PaperMetrics_Reset();
			MET_reset = 1;
		}
		HAL_UART_Receive_IT(&huart2, &paper_uart_rx_byte, 1);
		return;
	}
	if(huart->Instance == UART4){
		//HAL_UART_Transmit(&huart2, &RxSingleByte, 1, 10);

	}

	 if (huart->Instance == USART3) {
	        if (uart3_controller_index == 0) {
	            if (uart3_controller_byte == 0xAA) {
	                uart3_controller_buf[uart3_controller_index++] = uart3_controller_byte;
	            }
	            // Else ignore random bytes
	        } else {
	            uart3_controller_buf[uart3_controller_index++] = uart3_controller_byte;
	            if (uart3_controller_index >= UART3_CONTROLLER_PACKET_LEN) {
	                uart3_controller_index = 0;
	                uart3_controller_packet_ready = 1;
	            }
	        }
	        HAL_UART_Receive_IT(&huart3, &uart3_controller_byte, 1);  // Re-arm
	    }
}
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
	if (huart->Instance == UART5) {
		DDSM115_UART_TxCpltCallback(huart);
		return;
	}
	PaperMetrics_UART_TxCpltCallback(huart);
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
	if (huart->Instance == UART5) {
		DDSM115_UART_ErrorCallback(huart);
	}
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == BNO080_INT_Pin) {
        BNO08x_EXTI_Callback(GPIO_Pin);
    }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if (htim->Instance == TIM3)
	{
		// Every 10ms (100Hz)
		// Set the flag for CAN transmission
		//isCANReady = true;
	}
	if (htim->Instance == TIM4)
	{
		PaperMetrics_OnControlRelease();
		isDDSM115Ready = true;
		isCYBERGEARReady = true;
	}

}











/**
  * @brief Updates continuous base ANGLE (phi) and ANGULAR VELOCITY (phi_dot)
  *        AND linear position (x) and linear velocity (x_dot) based on DDSM115 feedback.
  *        Handles position wrap-around. Only processes data from motor 0x10.
  * @param Buffer: Pointer to the received RS485 data buffer.
  * @param current_phi_rad: Pointer to global variable storing continuous angular position (rad).
  * @param current_phi_dot_rad_s: Pointer to global variable storing angular velocity (rad/s).
  * @param current_x: Pointer to global variable storing linear position (m).
  * @param current_x_dot: Pointer to global variable storing linear velocity (m/s).
  * @retval None (Results are stored in the global variables via pointers).
  */

// ===== ANN INIT =====
void ANN_Init(void)
{
    ai_error err;
    ai_handle act_addr[] = { (ai_handle)nn_activations_pool };

#if ACTIVE_NN_POLICY == POLICY_POINT_MLP
    err = ai_nn_arm_a_point_create_and_init(&nn_policy, act_addr, NULL);
    if (err.type != AI_ERROR_NONE) {
        Error_Handler();
    }
    nn_input  = ai_nn_arm_a_point_inputs_get(nn_policy, NULL);
    nn_output = ai_nn_arm_a_point_outputs_get(nn_policy, NULL);
    nn_input[0].data  = (ai_handle)nn_in_obs;
    nn_output[0].data = (ai_handle)nn_out_action;
#elif ACTIVE_NN_POLICY == POLICY_RANGE_MLP
    err = ai_nn_arm_a_range_create_and_init(&nn_policy, act_addr, NULL);
    if (err.type != AI_ERROR_NONE) {
        Error_Handler();
    }
    nn_input  = ai_nn_arm_a_range_inputs_get(nn_policy, NULL);
    nn_output = ai_nn_arm_a_range_outputs_get(nn_policy, NULL);
    nn_input[0].data  = (ai_handle)nn_in_obs;
    nn_output[0].data = (ai_handle)nn_out_action;
#elif ACTIVE_NN_POLICY == POLICY_RANGE_GRU
    err = ai_nn_arm_c_range_gru_create_and_init(&nn_policy, act_addr, NULL);
    if (err.type != AI_ERROR_NONE) {
        Error_Handler();
    }
    nn_input  = ai_nn_arm_c_range_gru_inputs_get(nn_policy, NULL);
    nn_output = ai_nn_arm_c_range_gru_outputs_get(nn_policy, NULL);
    nn_input[0].data  = (ai_handle)nn_in_obs;
    nn_input[1].data  = (ai_handle)nn_gru_h;
    nn_output[0].data = (ai_handle)nn_out_action;
    nn_output[1].data = (ai_handle)nn_gru_h_out;
    memset(nn_gru_h, 0, sizeof(nn_gru_h));
    memset(nn_gru_h_out, 0, sizeof(nn_gru_h_out));
#endif
}

// ===== ANN INFERENCE =====
// IMU source: BNO08x wired directly to I2C3 (bno08x.h/.c), fused into
// body_pitch_rad/body_roll_rad/body_yaw_rad and their rates by
// StateEstimator_UpdateFromBNO() (called once per main-loop tick, before this).
// pitch/roll come from atan2() on body-frame gravity per STM32_DEPLOYMENT.md; yaw
// comes from the BNO's game rotation vector (no magnetometer - drifts slowly over
// long runs, see StateEstimator.h). The BNO's physical mounting orientation on the
// chassis has NOT been bench-verified against this axis mapping - tip the robot and
// watch LOG_obs[2]/[11] move the expected direction before trusting balance behavior.
static void NN_ReadImu(float *pitch, float *pitch_rate, float *roll, float *roll_rate,
                        float *yaw, float *yaw_rate)
{
    *pitch      = body_pitch_rad;
    *pitch_rate = body_pitch_rate_rad_s;
    *roll       = body_roll_rad;
    *roll_rate  = body_roll_rate_rad_s;
    *yaw        = body_yaw_rad;
    // Training uses root_ang_vel_w[:, 2], so observation 6 must use world Z.
    *yaw_rate   = body_yaw_rate_world_rad_s;
}

// ----- NNDriveFixedStance policy: 13 obs -> 2 actions (left/right DDSM115 wheel current,
// A). Legs are NOT policy-controlled (held at a fixed pose from the main loop). No
// joystick yet - DBG_desired_velocity_mps/DBG_desired_yawrate_radps (STM Studio-tunable)
// stand in for v_joy/w_joy, run through the exact same slew-limit + integration pipeline
// STM32_DEPLOYMENT.md specifies. -----
void ANN_Run(void)
{
    static bool  refs_valid   = false;
    static float pos_ref_m    = 0.0f;
    static float yaw_ref_rad  = 0.0f;
    static float v_cmd_mps    = 0.0f;
    static float w_cmd_radps  = 0.0f;
    static float x_odom_ref_m = 0.0f;

    if (isFallen() && !isStartupStrategy) {
        refs_valid = false;   // forces re-latch (incl. GRU h zero) on recovery
        prev_wheel_current_A[0] = 0.0f;
        prev_wheel_current_A[1] = 0.0f;
        ddsm_NN_current_command[0] = 0.0f;
        ddsm_NN_current_command[1] = 0.0f;
        return;
    }

	uint32_t observation_start = PaperMetrics_CycleNow();

    // ---- odometry (obs[0]/[1]) ----
    float phi_left  = WHEEL_POS_SIGN_LEFT  * DDSM115MotorList[0].phi_rad;
    float phi_right = WHEEL_POS_SIGN_RIGHT * DDSM115MotorList[1].phi_rad;
    float omega_left  = WHEEL_VEL_SIGN_LEFT  * DDSM115MotorList[0].phi_dot_rad_s;
    float omega_right = WHEEL_VEL_SIGN_RIGHT * DDSM115MotorList[1].phi_dot_rad_s;

    float pitch_rad, pitch_rate_radps, roll_rad, roll_rate_radps, yaw_rad, yaw_rate_radps;
    NN_ReadImu(&pitch_rad, &pitch_rate_radps, &roll_rad, &roll_rate_radps, &yaw_rad, &yaw_rate_radps);

    if (testbench_policy_reset_requested != 0u) {
        refs_valid = false;
        prev_wheel_current_A[0] = 0.0f;
        prev_wheel_current_A[1] = 0.0f;
        testbench_policy_reset_requested = 0u;
    }

    if (!refs_valid) {
        x_odom_ref_m = WHEEL_RADIUS_R * 0.5f * (phi_left + phi_right);
        pos_ref_m = 0.0f;
        yaw_ref_rad = yaw_rad;
        v_cmd_mps = 0.0f;
        w_cmd_radps = 0.0f;
#if ACTIVE_NN_POLICY == POLICY_RANGE_GRU
        memset(nn_gru_h, 0, sizeof(nn_gru_h));
#endif
        refs_valid = true;
    }

    float x_odom_m = WHEEL_RADIUS_R * 0.5f * (phi_left + phi_right) - x_odom_ref_m;
    float velocity_mps = WHEEL_RADIUS_R * 0.5f * (omega_left + omega_right);

    // ---- debug-command pipeline (replaces joystick, exact spec slew/integration) ----
    float dt = NNDRIVE_TICK_DT_S;
    v_cmd_mps   += clamp_float(DBG_desired_velocity_mps  - v_cmd_mps,  -1.0f * dt, 1.0f * dt);
    w_cmd_radps += clamp_float(DBG_desired_yawrate_radps - w_cmd_radps, -4.0f * dt, 4.0f * dt);
    pos_ref_m   += v_cmd_mps * dt;
    yaw_ref_rad  = wrap_pi(yaw_ref_rad + w_cmd_radps * dt);   // wrap for numerical hygiene only

    float raw_pos_err = x_odom_m - pos_ref_m;
    float pos_err_m = clamp_float(raw_pos_err, -0.5f, 0.5f);
    pos_ref_m += raw_pos_err - pos_err_m;              // back-calculation anti-windup

    float yaw_diff = yaw_rad - yaw_ref_rad;             // RAW, unwrapped - no clamp needed
    float yaw_err_sin = sinf(yaw_diff);
    float yaw_err_cos = cosf(yaw_diff);

    // ---- assemble 13 obs, exact STM32_DEPLOYMENT.md order ----
    nn_in_obs[0]  = pos_err_m / 0.5f;
    nn_in_obs[1]  = velocity_mps / 1.0f;
    nn_in_obs[2]  = pitch_rad / 0.43633231f;   // 25deg in rad
    nn_in_obs[3]  = pitch_rate_radps / 4.0f;
    nn_in_obs[4]  = yaw_err_sin;
    nn_in_obs[5]  = yaw_err_cos;
    nn_in_obs[6]  = yaw_rate_radps / 4.0f;
    nn_in_obs[7]  = v_cmd_mps / 1.0f;
    nn_in_obs[8]  = w_cmd_radps / 2.0f;
    nn_in_obs[9]  = prev_wheel_current_A[0] / 2.0f;
    nn_in_obs[10] = prev_wheel_current_A[1] / 2.0f;
    nn_in_obs[11] = roll_rad / 0.43633231f;
    nn_in_obs[12] = roll_rate_radps / 4.0f;

    for (uint32_t i = 0; i < NNDRIVE_OBS_DIM; i++) {
        if (!isfinite(nn_in_obs[i])) {
            nn_in_obs[i] = 0.0f;
        } else if (nn_in_obs[i] > 10.0f) {
            nn_in_obs[i] = 10.0f;
        } else if (nn_in_obs[i] < -10.0f) {
            nn_in_obs[i] = -10.0f;
        }
    }
	PaperMetrics_RecordDuration(PAPER_Q_OBSERVATION, observation_start);

    // ---- run network ----
	uint32_t inference_start = PaperMetrics_CycleNow();
#if ACTIVE_NN_POLICY == POLICY_RANGE_GRU
    ai_i32 batch = ai_nn_arm_c_range_gru_run(nn_policy, nn_input, nn_output);
#elif ACTIVE_NN_POLICY == POLICY_RANGE_MLP
    ai_i32 batch = ai_nn_arm_a_range_run(nn_policy, nn_input, nn_output);
#else
    ai_i32 batch = ai_nn_arm_a_point_run(nn_policy, nn_input, nn_output);
#endif
	PaperMetrics_RecordDuration(PAPER_Q_INFERENCE, inference_start);
	uint32_t action_start = PaperMetrics_CycleNow();
    bool inference_valid =
        (batch == 1) && isfinite(nn_out_action[0]) && isfinite(nn_out_action[1]);
#if ACTIVE_NN_POLICY == POLICY_RANGE_GRU
    for (uint32_t i = 0; i < NNDRIVE_GRU_HIDDEN && inference_valid; i++) {
        inference_valid = isfinite(nn_gru_h_out[i]);
    }
#endif
    if (!inference_valid) {
        ddsm_NN_current_command[0] = 0.0f;
        ddsm_NN_current_command[1] = 0.0f;
		prev_wheel_current_A[0] = 0.0f;
		prev_wheel_current_A[1] = 0.0f;
#if ACTIVE_NN_POLICY == POLICY_RANGE_GRU
        memset(nn_gru_h, 0, sizeof(nn_gru_h));
        memset(nn_gru_h_out, 0, sizeof(nn_gru_h_out));
#endif
		PaperMetrics_RecordDuration(PAPER_Q_ACTION, action_start);
        return;
    }
#if ACTIVE_NN_POLICY == POLICY_RANGE_GRU
    memcpy(nn_gru_h, nn_gru_h_out, sizeof(nn_gru_h));   // thread forward, never re-zero per-tick
#endif

    // ---- actions: raw ONNX output IS the logical current, no scaling ----
    float logical_current_left_A = clamp_float(nn_out_action[0],
	                                               -NNDRIVE_CURRENT_LIMIT_A,
	                                               NNDRIVE_CURRENT_LIMIT_A);
    float logical_current_right_A = clamp_float(nn_out_action[1],
	                                                -NNDRIVE_CURRENT_LIMIT_A,
	                                                NNDRIVE_CURRENT_LIMIT_A);

    // Convert logical policy directions to the physical motor/wiring directions
    // only at the actuator boundary. On this chassis the physical yaw response
    // is opposite the simulation convention, so cross the two destinations.
    ddsm_NN_current_command[0] = DDSM_CURRENT_SIGN_LEFT  * logical_current_right_A;
    ddsm_NN_current_command[1] = DDSM_CURRENT_SIGN_RIGHT * logical_current_left_A;

    // Training feeds back CurrentActionProcessor.command_current before the
    // mirrored-wheel physical torque sign or physical destination swap is applied.
    prev_wheel_current_A[0] = logical_current_left_A;
    prev_wheel_current_A[1] = logical_current_right_A;
	PaperMetrics_RecordDuration(PAPER_Q_ACTION, action_start);
}
static float clamp_float(float value, float min_value, float max_value)
{
    if (value < min_value) {
        return min_value;
    }
    if (value > max_value) {
        return max_value;
    }
    return value;
}

static float wrap_pi(float angle_rad)
{
    while (angle_rad > 3.14159265f) {
        angle_rad -= 6.28318531f;
    }
    while (angle_rad < -3.14159265f) {
        angle_rad += 6.28318531f;
    }
    return angle_rad;
}

// Updates all MET_* bench-metric variables from the current cycle's sensor/command
// values. Call once per 15ms control tick. See the MET_ globals above for what each
// one means and which metrics_per_run_template.csv column it maps to.
static void MET_Update(void)
{
    static bool  running         = false; // true once MET_reset has fired at least once
    static bool  have_init_theta = false;
    static float sum_theta_sq    = 0.0f;
    static float sum_I_sq        = 0.0f;
    static uint32_t sample_count = 0;
    static float x_ref_m         = 0.0f;
    static bool  disturbed       = false;
    static bool  recovered       = false;
    static float settle_timer_s  = 0.0f;

    if (MET_reset) {
        MET_reset = 0;

        have_init_theta = false;
        sum_theta_sq    = 0.0f;
        sum_I_sq        = 0.0f;
        sample_count    = 0;
        disturbed       = false;
        recovered       = false;
        settle_timer_s  = 0.0f;
        x_ref_m = WHEEL_RADIUS_R * 0.5f *
                  (WHEEL_POS_SIGN_LEFT  * DDSM115MotorList[0].phi_rad +
                   WHEEL_POS_SIGN_RIGHT * DDSM115MotorList[1].phi_rad);

        MET_theta_init_deg    = 0.0f;
        MET_theta_max_abs_deg = 0.0f;
        MET_theta_rms_deg     = 0.0f;
        MET_I_rms_A           = 0.0f;
        MET_t_rec_s           = 0.0f;
        MET_x_final_m         = 0.0f;
        MET_run_time_s        = 0.0f;

        running = true;
    }

    if (!running) {
        return; // wait for the first MET_reset before accumulating anything
    }

    float pitch_rad, pitch_rate_radps, roll_rad, roll_rate_radps, yaw_rad, yaw_rate_radps;
    NN_ReadImu(&pitch_rad, &pitch_rate_radps, &roll_rad, &roll_rate_radps, &yaw_rad, &yaw_rate_radps);
    float theta_deg = pitch_rad * MET_RAD2DEG;
    float abs_theta_deg = fabsf(theta_deg);
    MET_theta_deg = theta_deg;

    if (!have_init_theta) {
        MET_theta_init_deg = theta_deg;
        have_init_theta = true;
    }

    if (abs_theta_deg > MET_theta_max_abs_deg) {
        MET_theta_max_abs_deg = abs_theta_deg;
    }

    float x_m = WHEEL_RADIUS_R * 0.5f *
                (WHEEL_POS_SIGN_LEFT  * DDSM115MotorList[0].phi_rad +
                 WHEEL_POS_SIGN_RIGHT * DDSM115MotorList[1].phi_rad) - x_ref_m;
    MET_x_final_m = x_m;

    float i_left_A = DDSM115MotorList[0].current_feedback_A;
    float i_right_A = DDSM115MotorList[1].current_feedback_A;

    sample_count++;
    sum_theta_sq += theta_deg * theta_deg;
    sum_I_sq     += 0.5f * (i_left_A * i_left_A + i_right_A * i_right_A);
    MET_theta_rms_deg = sqrtf(sum_theta_sq / (float)sample_count);
    MET_I_rms_A       = sqrtf(sum_I_sq     / (float)sample_count);

    MET_run_time_s += MET_SAMPLE_DT_S;

    // Recovery time: latch the elapsed time once |theta| has gone above the
    // "disturbed" threshold and then stayed below the (lower) "settled"
    // threshold continuously for MET_SETTLE_HOLD_S. Stays at 0 if the run
    // never leaves the disturbed threshold (e.g. undisturbed_balance).
    if (!recovered) {
        if (abs_theta_deg > MET_THETA_DISTURBED_DEG) {
            disturbed = true;
            settle_timer_s = 0.0f;
        }
        if (disturbed) {
            MET_t_rec_s = MET_run_time_s;
            if (abs_theta_deg < MET_THETA_SETTLED_DEG) {
                settle_timer_s += MET_SAMPLE_DT_S;
                if (settle_timer_s >= MET_SETTLE_HOLD_S) {
                    recovered = true; // MET_t_rec_s is now frozen
                }
            } else {
                settle_timer_s = 0.0f;
            }
        }
    }
}

// Updates all LOG_* per-tick telemetry variables. Call once per 15ms control tick,
// after ANN_Run() so nn_in_obs/nn_out_action/ddsm_NN_current_command reflect this
// cycle's values. See the LOG_ globals above for the timeseries_log_template.csv
// column mapping.
static void LOG_Update(void)
{
    float pitch_rad, pitch_rate_radps, roll_rad, roll_rate_radps, yaw_rad, yaw_rate_radps;
    NN_ReadImu(&pitch_rad, &pitch_rate_radps, &roll_rad, &roll_rate_radps, &yaw_rad, &yaw_rate_radps);
    LOG_theta_deg = pitch_rad * MET_RAD2DEG;

    for (uint32_t i = 0; i < NNDRIVE_OBS_DIM; i++) {
        LOG_obs[i] = nn_in_obs[i];
    }

    LOG_action_L = nn_out_action[0];
    LOG_action_R = nn_out_action[1];

    LOG_I_L_cmd_A = ddsm_NN_current_command[0];
    LOG_I_R_cmd_A = ddsm_NN_current_command[1];

    LOG_I_L_meas_A = DDSM115MotorList[0].current_feedback_A;
    LOG_I_R_meas_A = DDSM115MotorList[1].current_feedback_A;
}

static void PAPER_SendTelemetry(void)
{
	PaperTelemetryState sample = {0};
	float pitch_rad, pitch_rate_radps, roll_rad, roll_rate_radps, yaw_rad, yaw_rate_radps;
	NN_ReadImu(&pitch_rad, &pitch_rate_radps, &roll_rad, &roll_rate_radps,
	           &yaw_rad, &yaw_rate_radps);

	if (bno_data_valid) sample.flags |= (1u << 0);
	if (isFallen()) sample.flags |= (1u << 1);
	if (isStartupStrategy) sample.flags |= (1u << 2);
	if (DBG_testbench_running) sample.flags |= (1u << 3);
	if (DBG_testbench_waiting) sample.flags |= (1u << 4);
	if (DBG_testbench_recovering) sample.flags |= (1u << 5);
	sample.flags |= ((uint32_t)ACTIVE_NN_POLICY & 0x03u) << 8;
	sample.flags |= ((uint32_t)DBG_testbench_stage & 0xFFu) << 16;
	sample.flags |= ((uint32_t)DBG_testbench_fall_count & 0xFFu) << 24;

	sample.attitude[0] = pitch_rad;
	sample.attitude[1] = pitch_rate_radps;
	sample.attitude[2] = roll_rad;
	sample.attitude[3] = roll_rate_radps;
	sample.attitude[4] = yaw_rad;
	// Keep the legacy CSV column as raw body gyro Z; diagnostic[0] below is the
	// world-Z value actually consumed by NN observation 6.
	sample.attitude[5] = body_yaw_rate_rad_s;

	float phi_left = WHEEL_POS_SIGN_LEFT * DDSM115MotorList[0].phi_rad;
	float phi_right = WHEEL_POS_SIGN_RIGHT * DDSM115MotorList[1].phi_rad;
	float omega_left = WHEEL_VEL_SIGN_LEFT * DDSM115MotorList[0].phi_dot_rad_s;
	float omega_right = WHEEL_VEL_SIGN_RIGHT * DDSM115MotorList[1].phi_dot_rad_s;
	sample.diagnostic[0] = body_yaw_rate_world_rad_s;
	sample.diagnostic[1] = body_yaw_rate_quat_rad_s;
	sample.diagnostic[2] = omega_left;
	sample.diagnostic[3] = omega_right;
	sample.position_m = WHEEL_RADIUS_R * 0.5f * (phi_left + phi_right);
	sample.velocity_mps = WHEEL_RADIUS_R * 0.5f * (omega_left + omega_right);

	for (uint32_t i = 0; i < PAPER_METRICS_WHEEL_COUNT; i++) {
		sample.wheel_position_rad[i] = DDSM115MotorList[i].phi_rad;
		sample.wheel_velocity_radps[i] = DDSM115MotorList[i].phi_dot_rad_s;
		sample.wheel_current_command_A[i] = ddsm_NN_current_command[i];
		sample.wheel_current_measured_A[i] = DDSM115MotorList[i].current_feedback_A;
	}
	for (uint32_t i = 0; i < PAPER_METRICS_LEG_COUNT; i++) {
		sample.leg_angle_rad[i] = CyberGearMotorList[i].angle;
	}
	for (uint32_t i = 0; i < NNDRIVE_OBS_DIM; i++) {
		sample.observation[i] = nn_in_obs[i];
	}
	for (uint32_t i = 0; i < NNDRIVE_ACT_DIM; i++) {
		sample.action[i] = nn_out_action[i];
	}

	sample.run_value[0] = MET_theta_deg;
	sample.run_value[1] = MET_theta_init_deg;
	sample.run_value[2] = MET_theta_max_abs_deg;
	sample.run_value[3] = MET_theta_rms_deg;
	sample.run_value[4] = MET_I_rms_A;
	sample.run_value[5] = MET_t_rec_s;
	sample.run_value[6] = MET_x_final_m;
	sample.run_value[7] = MET_run_time_s;
	PaperMetrics_Send(&sample);
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
