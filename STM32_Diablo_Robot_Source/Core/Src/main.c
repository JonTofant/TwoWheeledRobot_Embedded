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
#include "telemetry.h"
#include "controler.h"
#include "joystick.h"
#include "StartupStrategy.h"
#include "StateEstimator.h"
#include "state_machine.h"
#include "JumpStrategy.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */



/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

// ESP32 buffer size (ROLL PITCH YAW)
#define PACKET_SIZE 19

// Set to 1 to re-enable the UART2 debug telemetry print to the ESP32.
// Disabled by default to save loop time for the 15ms ANN/DDSM115 update.
#define ENABLE_UART2_DEBUG_TELEMETRY 0



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

uint8_t RS485_RxBuffer[RS485_BUFFER_SIZE];

// ===== AI network globals =====
STAI_NETWORK_CONTEXT_DECLARE(ann_network_context, STAI_NETWORK_CONTEXT_SIZE)
STAI_ALIGNED(32) static uint8_t ann_activations[STAI_NETWORK_ACTIVATION_1_SIZE_BYTES];
STAI_ALIGNED(4) static float ann_in_data[STAI_NETWORK_IN_1_SIZE];
STAI_ALIGNED(4) static float ann_out_data[STAI_NETWORK_OUT_1_SIZE];
static stai_ptr ann_input[STAI_NETWORK_IN_NUM] = { (stai_ptr)ann_in_data };
static stai_ptr ann_output[STAI_NETWORK_OUT_NUM] = { (stai_ptr)ann_out_data };

// ===== NN Drive policy (policy_drive, 18 obs -> 6 actions) =====
// 1 = new NN-drive policy owns CyberGear leg targets + wheel current; 0 = revert to the
// old posture_controler()/hardcoded-angle leg path and the old 8-obs/2-action ANN_Run(),
// both kept intact under #else/#if !NN_DRIVE_POLICY_ACTIVE for instant rollback.
#define NN_DRIVE_POLICY_ACTIVE 1

#if !NN_DRIVE_POLICY_ACTIVE
static float prev_action_left  = 0.0f;
static float prev_action_right = 0.0f;
#endif

#define WHEEL_POS_SIGN_LEFT        (-1.0f)
#define WHEEL_POS_SIGN_RIGHT       (+1.0f)
#define WHEEL_VEL_SIGN_LEFT        (-1.0f)
#define WHEEL_VEL_SIGN_RIGHT       (+1.0f)

#define PITCH_SIGN                 (+1.0f)
#define PITCH_RATE_SIGN            (+1.0f)
#define YAW_SIGN                   (-1.0f)
#define YAW_RATE_SIGN              (-1.0f)

#define DDSM_CURRENT_SIGN_LEFT     (+1.0f)
#define DDSM_CURRENT_SIGN_RIGHT    (-1.0f)
#define STM32_TEST_CURRENT_LIMIT_A (1.0f)   // was 0.5f - raised for the NN-drive policy's first bench port

// Normalization/scale constants (Template-Twowheeledrobot-NNDrive-v0 spec, STM32_DEPLOYMENT.md)
#define NNDRIVE_POS_ERR_NORM_M       (0.5f)   // obs[0]: pos_err / 0.5m  (NOTE: differs from old policy's /1.0f)
#define NNDRIVE_YAW_ERR_NORM_RAD     (1.5f)   // obs[4]: yaw_err / 1.5 rad (NEW - was /pi in the old policy)
#define NNDRIVE_VEL_CMD_NORM_MPS     (1.0f)
#define NNDRIVE_YAWRATE_CMD_NORM_RPS (2.0f)
#define NNDRIVE_CG_POS_NORM_RAD      (0.45f)  // obs[8-11]: cg_pos / 0.45 rad
#define NNDRIVE_PREV_CURRENT_NORM_A  (2.0f)   // obs[12-13]: prev_current / 2.0A (trained range, not the firmware clamp)
#define CG_ACTION_SCALE_RAD          (0.45f)  // ONNX CG outputs are tanh(a)*this; divide to recover tanh-space prev action

// Joystick is assumed to send zero for this port - no reference integration implemented yet.
#define NNDRIVE_VELOCITY_CMD_MPS  (0.0f)      // obs[6]
#define NNDRIVE_YAWRATE_CMD_RPS   (0.0f)      // obs[7]

// CyberGear index <-> physical corner mapping - CONFIRMED via physical motor ID check
// (motorID decimal 30/31/21/20 = 0x1E/0x1F/0x15/0x14 = CyberGearMotorList[0..3]):
// FR=21(0x15)->idx2, BR=20(0x14)->idx3, FL=31(0x1F)->idx1, BL=30(0x1E)->idx0.
// This matches cybergear.c's own leftFront/leftBack angle-limit naming (LB=0, LF=1) and
// system_init.c, NOT telemetry.c/DDSM115.h's MOTOR_CG_LF/LB aliases (LF=0,LB=1), which
// are therefore wrong and should not be trusted for CyberGearMotorList indexing.
// The CyberGear_IndexBenchTest() below remains available to re-verify after any rewiring.
#define NNDRIVE_CG_IDX_FL 1   // ann_out_data[0] -> CyberGearMotorList[1]
#define NNDRIVE_CG_IDX_FR 2   // ann_out_data[1] -> CyberGearMotorList[2]
#define NNDRIVE_CG_IDX_BL 0   // ann_out_data[2] -> CyberGearMotorList[0]
#define NNDRIVE_CG_IDX_BR 3   // ann_out_data[3] -> CyberGearMotorList[3]

// Per-corner hard position limits (deg->rad): fl/br in [-10,+90]deg, fr/bl in [-90,+10]deg
#define NNDRIVE_CG_FLBR_MIN_RAD (-10.0f * 0.017453293f)
#define NNDRIVE_CG_FLBR_MAX_RAD ( 90.0f * 0.017453293f)
#define NNDRIVE_CG_FRBL_MIN_RAD (-90.0f * 0.017453293f)
#define NNDRIVE_CG_FRBL_MAX_RAD ( 10.0f * 0.017453293f)

#define NNDRIVE_CG_SLEW_RATE_RADPS (3.0f)   // matches spec's slew-limit requirement
#define NNDRIVE_TICK_DT_S         (0.015f)  // 15ms TIM4 tick (matches MET_SAMPLE_DT_S)

// CyberGear rest-offset: at boot setMechanicalZero is called while each leg rests at its
// FOLDED hard stop, so firmware .angle=0 there. In the sim/policy frame that folded pose is
// NOT 0deg - it is the small-magnitude joint limit: fl/br fold to -10deg (min of [-10,+90]),
// fr/bl fold to +10deg (max of [-90,+10]). We latch the measured .angle at rest as a per-corner
// reference and work in sim frame: sim_angle = (.angle - cg_angle_ref) + rest; convert back to
// firmware only when writing desired_angle. The obs difference is immune to a constant bias
// in the .angle CAN decode, but the COMMAND (cg_cmd_fw = ref + ...) is NOT: cg_angle_ref must
// be in the exact frame Motor_SendMITCommand encodes desired_angle in. The CAN RX decode is
// therefore offset-free (the old +M_PI_2 term there drove every leg 90deg into its hard stop).
// Set NNDRIVE_CG_REST_ENABLE=0 to fall back to raw .angle behaviour.
#define NNDRIVE_CG_REST_ENABLE   1
#define NNDRIVE_CG_REST_FLBR_RAD (-10.0f * 0.017453293f)  // fl, br folded-rest sim angle
#define NNDRIVE_CG_REST_FRBL_RAD ( 10.0f * 0.017453293f)  // fr, bl folded-rest sim angle

#if NN_DRIVE_POLICY_ACTIVE
static float prev_cg_action[4]       = {0.0f, 0.0f, 0.0f, 0.0f}; // tanh-space obs[14-17], order fl/fr/bl/br
static float prev_wheel_current_A[2] = {0.0f, 0.0f};             // obs[12-13], order left/right
static float cg_last_cmd_rad[4]      = {0.0f, 0.0f, 0.0f, 0.0f}; // slew memory (SIM frame), indexed like CyberGearMotorList[]
static float cg_cmd_fw[4]            = {0.0f, 0.0f, 0.0f, 0.0f}; // firmware-frame target the main loop writes to desired_angle
static float cg_angle_ref[4]         = {0.0f, 0.0f, 0.0f, 0.0f}; // firmware .angle latched at folded rest, per motor index (STM Studio readback)
#if NNDRIVE_CG_REST_ENABLE
// sim-frame folded-rest angle per motor index (0=BL,1=FL,2=FR,3=BR): BL/FR=+10, FL/BR=-10.
static const float cg_sim_at_rest[4] = { NNDRIVE_CG_REST_FRBL_RAD,   // idx0 = BL
                                         NNDRIVE_CG_REST_FLBR_RAD,   // idx1 = FL
                                         NNDRIVE_CG_REST_FRBL_RAD,   // idx2 = FR
                                         NNDRIVE_CG_REST_FLBR_RAD }; // idx3 = BR
// Per-corner hardware<->sim rotation-direction sign (motor-index order 0=BL,1=FL,2=FR,3=BR),
// applied to BOTH obs and command so the loop stays coherent:
//   obs: sim = dir*(.angle - ref) + rest ;   cmd: .desired_angle = ref + dir*(sim_target - rest)
// The earlier all -1 setting was derived from bench runs corrupted by the +M_PI_2 decode
// offset (every target was ~90deg off, so "wrong way" observations meant nothing). Re-derive
// per corner with CyberGear_IndexBenchTest() now that the frames agree: a +0.12 rad nudge on
// index i should move that corner the way a positive sim angle does (fl/br extend toward
// +90deg, fr/bl toward their -90deg side is NEGATIVE sim direction). Flip only proven corners.
static const float cg_dir[4]         = { +1.0f, +1.0f, +1.0f, +1.0f };
#endif
static bool  cg_ref_captured         = false;                    // latch cg_angle_ref once (boot rest); NOT reset on fall
static bool  cg_slew_initialized     = false;                    // slew seed latch; reset on fall so it re-seeds from current pose
// Write CG_REF_recapture=1 in STM Studio (with legs at folded rest) to force a fresh cg_angle_ref
// capture - use if the first-tick capture happened before valid CAN angle feedback arrived, or to
// re-zero after manually posing the legs. Clears itself once applied.
volatile uint8_t CG_REF_recapture    = 0;
#endif

// One-shot bench routine to verify the NNDRIVE_CG_IDX_* mapping above against the real
// robot before trusting closed-loop leg control. Write CG_BENCH_start = 1 (e.g. via STM
// Studio) with the robot hand-held/on a stand (not standing on its own wheels) to run it.
#define CYBERGEAR_INDEX_BENCH_TEST 1
#if CYBERGEAR_INDEX_BENCH_TEST
volatile uint8_t CG_BENCH_start = 0;  // write 1 to begin the sweep; clears itself when done
volatile int8_t  CG_BENCH_active_index = -1; // -1 = idle, else 0..3 = CyberGearMotorList index currently being tested
#endif

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
// obs_0..obs_17 are the exact 18 floats fed to the network (ann_in_data, normalized) -
// adjust LOG_Update() if your training env logged obs in a different order/units.
// action_L/R are the network's raw wheel-current output (pre sign-flip, pre safety
// clamp); action_cg_fl/fr/bl/br are the raw CyberGear position outputs (pre clamp/slew).
// I_L/R_cmd_A and I_L/R_meas_A are both the same commanded current actually sent to
// the DDSM115 (post sign-flip and clamping) - there's no separate current feedback
// from the drive, so "measured" just mirrors "commanded".
volatile float   LOG_theta_deg      = 0.0f;  // instantaneous pitch (deg); first recorded sample -> initial_theta_deg
volatile float   LOG_obs[18]        = {0};   // -> obs_0..obs_17
volatile float   LOG_action_L       = 0.0f;  // -> action_L (raw wheel-current network output)
volatile float   LOG_action_R       = 0.0f;  // -> action_R (raw wheel-current network output)
volatile float   LOG_action_cg_fl   = 0.0f;  // raw CyberGear position output, front-left
volatile float   LOG_action_cg_fr   = 0.0f;  // raw CyberGear position output, front-right
volatile float   LOG_action_cg_bl   = 0.0f;  // raw CyberGear position output, back-left
volatile float   LOG_action_cg_br   = 0.0f;  // raw CyberGear position output, back-right
volatile float   LOG_I_L_cmd_A      = 0.0f;  // -> I_L_cmd_A
volatile float   LOG_I_R_cmd_A      = 0.0f;  // -> I_R_cmd_A
volatile float   LOG_I_L_meas_A     = 0.0f;  // -> I_L_meas_A
volatile float   LOG_I_R_meas_A     = 0.0f;  // -> I_R_meas_A

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
uint16_t getDMACurrentIndex(void);
volatile uint16_t rxReadIndex = 0;       // Your software read pointer






#define BNO080_PACKET_SIZE 25  // 1 SOF + 6×4-byte floats
#define UART1_RX_BUFFER_SIZE 128
static uint8_t  uart1RxBuffer[UART1_RX_BUFFER_SIZE];

#define UART2_RX_BUFFER_SIZE 10
static uint8_t uart2RxBuffer[UART2_RX_BUFFER_SIZE];

float ddsm_NN_current_command[2] = {0};

char NN_buffer[80] = {0};

void ANN_Init(void);
void ANN_Run(void);
static float clamp_float(float value, float min_value, float max_value);
static float wrap_pi(float angle_rad);
static void MET_Update(void);
static void LOG_Update(void);
#if CYBERGEAR_INDEX_BENCH_TEST
static void CyberGear_IndexBenchTest(void);
#endif


/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint8_t sendUart2Data = 0;
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
  //MX_X_CUBE_AI_Init();
  /* USER CODE BEGIN 2 */

  ANN_Init();

  /*char boot[60];
  sprintf(boot, "BOOT OK - NN init done\r\n");
  HAL_UART_Transmit(&huart3, (uint8_t*)boot, strlen(boot), 100);
  HAL_Delay(100);*/

  // Kick off DMA+Idle for USART1
  HAL_UARTEx_ReceiveToIdle_IT(&huart1,
                             uart1RxBuffer,
                             UART1_RX_BUFFER_SIZE);


  // Enable UART DMA for uart2
  HAL_UART_Receive_DMA(&huart2, uart2RxBuffer, UART2_RX_BUFFER_SIZE);



  // Enable UART DMA for uart4
  HAL_UARTEx_ReceiveToIdle_IT(&huart5, RS485_RxBuffer, RS485_BUFFER_SIZE);


  // ------ PRECISE SAMPLE TIME MEASUREMENTS ------

  // Enable TRC (Trace control) - needed to use DWT
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;

  // Reset the cycle counter
  DWT->CYCCNT = 0;

  // Enable the cycle counter
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

  //------------------------------------------------


// SEND QUERY TO DDSM115
  uint8_t cmd[10] = {0};
  cmd[0] = 0xAA;  // Motor ID
  cmd[1] = 0x64;
  cmd[2] = 0x00;  // High byte
  cmd[3] = 0x00;  // Low byte
  cmd[4] = 0x00;
  cmd[5] = 0x00;
  cmd[6] = 0x00;
  cmd[7] = 0x00;
  cmd[8] = 0x00;
  cmd[9] = compute_crc8(cmd, 9);
  // Set GPIO pin to transmit mode

  HAL_GPIO_WritePin(RS485_DIR_GPIO_Port, RS485_DIR_Pin, GPIO_PIN_SET);
  HAL_UART_Transmit(&huart5, cmd, 10, HAL_MAX_DELAY);
  HAL_Delay(10);  // Allow time for the command to be processed
  // Set GPIO pin to receive mode
  HAL_GPIO_WritePin(RS485_DIR_GPIO_Port, RS485_DIR_Pin, GPIO_PIN_RESET);




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

  	init_leg_state(&leg_state_rf);
  	init_leg_state(&leg_state_lf);
  	init_leg_state(&leg_state_rb);
  	init_leg_state(&leg_state_lb);


  	// Testing of inverse kinematics

  	//start TIM2 for USART2 data transfer
  		HAL_TIM_Base_Start_IT(&htim2);
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



	 if (isCANReady) {
		 isCANReady = false;
		 }

	 if (uart3_controller_packet_ready) {
		 process_joystick_input();
		 uart3_controller_packet_ready = 0;
	 }



#if NN_DRIVE_POLICY_ACTIVE
	 // Leg targets now come from the NN-drive policy, sent from the isDDSM115Ready
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
			 DDSM115setCurrent(0x10, 0);
			 HAL_Delay(2);
			 DDSM115setCurrent(0x11, 0);
		 }

		 if (isJumpStrategy) {
			 jump_strategy_control();
		 }

		 isCYBERGEARReady = false;
	 }
#else
	 if (isCYBERGEARReady){

		 if (isFallen())
		 {
			 if (startPressed == 1) isStartupStrategy = true;
		 }else{
			 if (xPressed == 1) isJumpStrategy = true;
		 }



		 if(isStartupStrategy)
		 {
			 startup_strategy_control();
		 }

		 if(isFallen()){
			 DDSM115setCurrent(0x10, 0);
			 HAL_Delay(2);
			 DDSM115setCurrent(0x11, 0);

		 }

		 if(isJumpStrategy)
		 {
			 jump_strategy_control();
		 }




		 	// ROBOT IS STANDING AND OPERATIONAL
		    if (!isFallen()) {
		    	if(isSTATIC){

		    	posture_controler();


		    	MOTOR_CG_LF.desired_angle = 0.0f;
		    	MOTOR_CG_LB.desired_angle = -0.0f;
		    	MOTOR_CG_RF.desired_angle = 0.0f;
		    	MOTOR_CG_RB.desired_angle = -0.0f;


		    	Motor_SendMITCommand(&MOTOR_CG_LF);
		    	Motor_SendMITCommand(&MOTOR_CG_LB);
		    	HAL_Delay(5);
		    	Motor_SendMITCommand(&MOTOR_CG_RF);
		    	Motor_SendMITCommand(&MOTOR_CG_RB);


				 isCYBERGEARReady = false;



		    	}
		    	// THE ROBOT IS IN LOCOMOTION STATE
		    	else if(isLOCOMOTION){
		    		// For now the same thing happens as in static

			    	posture_controler();


			    	Motor_SendMITCommand(&MOTOR_CG_LF);
			    	Motor_SendMITCommand(&MOTOR_CG_LB);
			    	HAL_Delay(5);
			    	Motor_SendMITCommand(&MOTOR_CG_RF);
			    	Motor_SendMITCommand(&MOTOR_CG_RB);


					 isCYBERGEARReady = false;
		    	}

		    }
		    else{

		   	 isCYBERGEARReady = false;

		    }
	 }
#endif // NN_DRIVE_POLICY_ACTIVE


	/* if (sendUart2Data){


		 sprintf(NN_buffer, "%d%d%.2f%.2f%.2f%.2f%.2f%.2f%.2f\r\n",
				 0xAA, 0x55, pitch_esp32, roll_esp32, gx_esp32, gy_esp32, gz_esp32, DDSM115MotorList[0].x_dot, DDSM115MotorList[1].x_dot);
		 sendUart2Data = 0;
		 HAL_UART_Transmit_DMA(&huart2, (uint8_t*)NN_buffer, strlen(NN_buffer));



		 if(!isFallen() || isStartupStrategy){

		 // calculate_cascaded_motor_currents(desired_v_left,desired_v_right, &current_motor1_out, &current_motor2_out, &total_force_out);

		 //current_motor1_out = 0.0f;
		 //current_motor2_out = 0.0f;

		 DDSM115setCurrent(0x11, -1* ddsm_NN_current_command[0]);
		 HAL_Delay(2);
		 DDSM115setCurrent(0x10, -1* ddsm_NN_current_command[1]);
		 }
		 else{}

		 isDDSM115Ready = false;
	 }*/

	 //ANN UPDATED SENDING FUNCTION
	 // Now gated on the 15ms TIM4 tick (isDDSM115Ready) instead of the 100ms
	 // TIM2 tick (sendUart2Data), so the policy and the DDSM115 current
	 // commands run at ~66.7Hz instead of 10Hz.
	 if (isDDSM115Ready) {
	     isDDSM115Ready = false;

#if CYBERGEAR_INDEX_BENCH_TEST
	     if (CG_BENCH_start || CG_BENCH_active_index >= 0) {
	         // Bench test owns the CyberGear motors this tick - skip the normal
	         // NN-drive policy entirely so nothing fights over MIT commands.
	         CyberGear_IndexBenchTest();
	     } else
#endif
	     {
	         // Run policy locally
	         ANN_Run();

	         // DEBUG: NNDRIVE_OUTPUT_DISABLE=1 stops all motor output (CyberGear MIT
	         // commands and DDSM115 currents) while still running ANN_Run()/MET_Update()/
	         // LOG_Update() every tick, so the raw network outputs can be inspected via
	         // STM Studio (LOG_action_cg_fl/fr/bl/br, LOG_obs[]) or a debugger watch on
	         // ann_out_data[]/cg_cmd_fw[] with the legs held still. Revert to 0 to resume
	         // normal operation.
#define NNDRIVE_OUTPUT_DISABLE 1
#if !NNDRIVE_OUTPUT_DISABLE
#if NN_DRIVE_POLICY_ACTIVE
	         if (!isFallen() || isStartupStrategy) {
	             // cg_cmd_fw[] is the firmware-frame target (sim target converted back via the
	             // folded-rest offset in ANN_Run); desired_angle is in the motor's own frame.
	             CyberGearMotorList[0].desired_angle = cg_cmd_fw[0];
	             CyberGearMotorList[1].desired_angle = cg_cmd_fw[1];
	             CyberGearMotorList[2].desired_angle = cg_cmd_fw[2];
	             CyberGearMotorList[3].desired_angle = cg_cmd_fw[3];

	             Motor_SendMITCommand(&CyberGearMotorList[0]);
	             Motor_SendMITCommand(&CyberGearMotorList[1]);
	             HAL_Delay(5);
	             Motor_SendMITCommand(&CyberGearMotorList[2]);
	             Motor_SendMITCommand(&CyberGearMotorList[3]);
	         }
	         // else: fallen and not in startup strategy - don't send new MIT commands
	         // this tick, legs hold their last state per the CyberGear's own timeout.
#endif

	         if (!isFallen() || isStartupStrategy) {
	             DDSM115setCurrent(0x11, 1 * ddsm_NN_current_command[0]);
	             HAL_Delay(5);
	             DDSM115setCurrent(0x10, 1 * ddsm_NN_current_command[1]);
	         }
#endif // !NNDRIVE_OUTPUT_DISABLE
	     }

	     // Bench metrics/telemetry for STM Studio (see MET_/LOG_ globals above), same 15ms tick.
	     MET_Update();
	     LOG_Update();
	 }

	 // Debug telemetry to ESP32, on the slower 100ms tick so UART2/DMA
	 // isn't loaded at the full 66.7Hz control rate.
	 // Disabled for now (ENABLE_UART2_DEBUG_TELEMETRY == 0) to free up loop
	 // time for the faster ANN/DDSM115 update above.
#if ENABLE_UART2_DEBUG_TELEMETRY
	 if (sendUart2Data) {
	     sendUart2Data = 0;

	     sprintf(NN_buffer, "%d%d %.2f %.2f %.2f %.2f %.2f %.2f %.2f | NN: %.3f %.3f\r\n",
	             0xAA, 0x55, pitch_esp32, roll_esp32, gx_esp32, gy_esp32, gz_esp32,
	             DDSM115MotorList[0].x_dot, DDSM115MotorList[1].x_dot,
	             ann_out_data[0], ann_out_data[1]);
	     HAL_UART_Transmit_DMA(&huart2, (uint8_t*)NN_buffer, strlen(NN_buffer));
	 }
#else
	 sendUart2Data = 0;
#endif


	 if (isTELEMETRYReady){
		 Send_Telemetry(&huart3);
	 }


    /* USER CODE END WHILE */

  //MX_X_CUBE_AI_Process();
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
  hi2c3.Init.ClockSpeed = 100000;
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
  huart1.Init.BaudRate = 115200;
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
  huart2.Init.BaudRate = 115200;
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
           // offset added here walks straight into cg_cmd_fw and shifts every leg target.
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
	if(huart->Instance == USART2){

		if(uart2RxBuffer[0] == 0xAA && uart2RxBuffer[1] == 0x55)
		{
			// Copy from buffer into current command array
			memcpy(ddsm_NN_current_command, uart2RxBuffer + 2, 8);

			// Re-arm DMA
			HAL_UART_Receive_DMA(&huart2, uart2RxBuffer, UART2_RX_BUFFER_SIZE);
		}

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

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
	   if (huart->Instance == UART5)
	    {
	        uint8_t motor_id = RS485_RxBuffer[0];

	        for (int i = 0; i < MAX_MOTORS_DDSM115; i++) {
	            if (DDSM115MotorList[i].motorID == motor_id) {
	            	update_ddsm115_state(&DDSM115MotorList[i], RS485_RxBuffer, 0.0505f);
	                break;
	            }
	        }

	        // Restart reception for next frame
	        HAL_UARTEx_ReceiveToIdle_IT(&huart5, RS485_RxBuffer, RS485_BUFFER_SIZE);
	    }
    if (huart->Instance == USART1)
    {
        // Size = number of bytes received into uart1RxBuffer[]
        uint16_t bytes = Size;
        uint16_t idx   = 0;

        // Process as many full packets as we have
        while (idx + PACKET_SIZE <= bytes)
        {
            if (uart1RxBuffer[idx] == 0xAA)
            {
                // Sync on first packet only
                if (!uartSynced)
                {
                    uartSynced = true;
                    // shift this good packet to buffer start if desired:
                    memmove(uart1RxBuffer,
                            uart1RxBuffer + idx,
                            PACKET_SIZE);
                    idx = 0;       // start parsing from buffer[0]
                    bytes = PACKET_SIZE;
                }

                // Parse the six floats (little-endian) at idx+1..idx+24
                float *f   = (float*)(uart1RxBuffer + idx + 1);
                yaw_esp32   = f[0];
                pitch_esp32 = f[1];
                roll_esp32  = f[2];
                gx_esp32    = f[3];
                gy_esp32    = f[4];
                gz_esp32    = f[5];

                // Advance past this packet
                idx += PACKET_SIZE;
            }
            else
            {
                // No SOF here, skip one byte
                idx++;
            }
        }



        // re-arm for next Idle
        HAL_UARTEx_ReceiveToIdle_IT(&huart1,
                                   uart1RxBuffer,
                                   UART1_RX_BUFFER_SIZE);
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
		isDDSM115Ready = true;
		isCYBERGEARReady = true;
		isTELEMETRYReady = true;

	}

	if (htim->Instance == TIM2) {
		sendUart2Data = 1;
		//HAL_GPIO_TogglePin(GPIOA, LED_BUILTIN_Pin);  // ← visible blink
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

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    // Check if the callback is from our telemetry UART.
    if (huart->Instance == USART3) {
        // The transmission is complete, so we set the flag to true,
        // allowing the next call to Send_Telemetry() to proceed.
        Telemetry_UART_TxCpltCallback(huart);
    }
}


// ===== ANN INIT =====
void ANN_Init(void)
{
    stai_ptr act_addr[] = { (stai_ptr)ann_activations };
    stai_return_code err = stai_runtime_init();
    if (err != STAI_SUCCESS) {
        Error_Handler();
    }

    err = user_stai_network_init(ann_network_context);
    if (err != STAI_SUCCESS) {
        Error_Handler();
    }

    err = stai_network_set_activations(ann_network_context, act_addr, STAI_NETWORK_ACTIVATIONS_NUM);
    if (err != STAI_SUCCESS) {
        Error_Handler();
    }

    err = stai_network_set_inputs(ann_network_context, ann_input, STAI_NETWORK_IN_NUM);
    if (err != STAI_SUCCESS) {
        Error_Handler();
    }

    err = stai_network_set_outputs(ann_network_context, ann_output, STAI_NETWORK_OUT_NUM);
    if (err != STAI_SUCCESS) {
        Error_Handler();
    }
}

// ===== ANN INFERENCE =====
#if !NN_DRIVE_POLICY_ACTIVE
// ----- Old policy: policy_current3, 8 obs -> 2 actions (wheel currents only) -----
void ANN_Run(void)
{
    static bool refs_valid = false;
    static float phi_left_ref_nn = 0.0f;
    static float phi_right_ref_nn = 0.0f;
    static float yaw_ref_nn_rad = 0.0f;

    if (isFallen() && !isStartupStrategy) {
        refs_valid = false;
        prev_action_left = 0.0f;
        prev_action_right = 0.0f;
        ddsm_NN_current_command[0] = 0.0f;
        ddsm_NN_current_command[1] = 0.0f;
        return;
    }

    float phi_left_nn = WHEEL_POS_SIGN_LEFT * DDSM115MotorList[0].phi_rad;
    float phi_right_nn = WHEEL_POS_SIGN_RIGHT * DDSM115MotorList[1].phi_rad;
    float omega_left_nn = WHEEL_VEL_SIGN_LEFT * DDSM115MotorList[0].phi_dot_rad_s;
    float omega_right_nn = WHEEL_VEL_SIGN_RIGHT * DDSM115MotorList[1].phi_dot_rad_s;
    float yaw_nn_rad = YAW_SIGN * yaw_esp32;

    if (!refs_valid) {
        phi_left_ref_nn = phi_left_nn;
        phi_right_ref_nn = phi_right_nn;
        yaw_ref_nn_rad = yaw_nn_rad;
        refs_valid = true;
    }

    float s_left_m = WHEEL_RADIUS_R * (phi_left_nn - phi_left_ref_nn);
    float s_right_m = WHEEL_RADIUS_R * (phi_right_nn - phi_right_ref_nn);
    float v_left_mps = WHEEL_RADIUS_R * omega_left_nn;
    float v_right_mps = WHEEL_RADIUS_R * omega_right_nn;

    float x_rel_m = 0.5f * (s_left_m + s_right_m);
    float x_dot_mps = 0.5f * (v_left_mps + v_right_mps);
    float theta_rad = PITCH_SIGN * roll_esp32;
    float theta_dot_radps = PITCH_RATE_SIGN * gx_esp32;
    float yaw_error_rad = wrap_pi(yaw_nn_rad - yaw_ref_nn_rad);
    float yaw_rate_radps = YAW_RATE_SIGN * gz_esp32;

    // Current policy input: normalized obs[8].
    ann_in_data[0] = x_rel_m / 1.0f;
    ann_in_data[1] = x_dot_mps / 1.0f;
    ann_in_data[2] = theta_rad / 0.43633231f;
    ann_in_data[3] = theta_dot_radps / 4.0f;
    ann_in_data[4] = yaw_error_rad / 3.14159265f;
    ann_in_data[5] = yaw_rate_radps / 4.0f;
    ann_in_data[6] = prev_action_left / 2.0f;
    ann_in_data[7] = prev_action_right / 2.0f;

    for (uint32_t i = 0; i < STAI_NETWORK_IN_1_SIZE; i++) {
        if (!isfinite(ann_in_data[i])) {
            ann_in_data[i] = 0.0f;
        } else if (ann_in_data[i] > 10.0f) {
            ann_in_data[i] = 10.0f;
        } else if (ann_in_data[i] < -10.0f) {
            ann_in_data[i] = -10.0f;
        }
    }

    if (stai_network_run(ann_network_context, STAI_MODE_SYNC) != STAI_SUCCESS) {
        ddsm_NN_current_command[0] = 0.0f;
        ddsm_NN_current_command[1] = 0.0f;
        return;
    }

    float left_nn_current_A = ann_out_data[0];
    float right_nn_current_A = ann_out_data[1];

    if (!isfinite(left_nn_current_A) || !isfinite(right_nn_current_A)) {
        ddsm_NN_current_command[0] = 0.0f;
        ddsm_NN_current_command[1] = 0.0f;
        return;
    }

    prev_action_left = left_nn_current_A;
    prev_action_right = right_nn_current_A;

    ddsm_NN_current_command[0] = clamp_float(DDSM_CURRENT_SIGN_LEFT * left_nn_current_A,
                                             -STM32_TEST_CURRENT_LIMIT_A,
                                             STM32_TEST_CURRENT_LIMIT_A);
    ddsm_NN_current_command[1] = clamp_float(DDSM_CURRENT_SIGN_RIGHT * right_nn_current_A,
                                             -STM32_TEST_CURRENT_LIMIT_A,
                                             STM32_TEST_CURRENT_LIMIT_A);
    //ddsm_NN_current_command[0] = 0.0f;
    //ddsm_NN_current_command[1] = 0.0f;

}

#else
// ----- New policy: policy_drive, 18 obs -> 6 actions (4 CyberGear leg targets + 2 wheel currents) -----
// Joystick is assumed to always send zero for this port (obs[6]/[7] hardcoded, no pos_ref/yaw_ref
// integration) - see STM32_DEPLOYMENT.md for the full spec this implements.
void ANN_Run(void)
{
    static bool refs_valid = false;
    static float phi_left_ref_nn = 0.0f;
    static float phi_right_ref_nn = 0.0f;
    static float yaw_ref_nn_rad = 0.0f;

    if (isFallen() && !isStartupStrategy) {
        refs_valid = false;
        cg_slew_initialized = false;
        prev_wheel_current_A[0] = 0.0f;
        prev_wheel_current_A[1] = 0.0f;
        prev_cg_action[0] = prev_cg_action[1] = prev_cg_action[2] = prev_cg_action[3] = 0.0f;
        ddsm_NN_current_command[0] = 0.0f;
        ddsm_NN_current_command[1] = 0.0f;
        // cg_last_cmd_rad[] is left untouched - the main loop won't send new MIT
        // commands while fallen (see the isDDSM115Ready block), so legs just hold.
        return;
    }

    // ---- obs 0-5: base/wheel, same math as the old policy ----
    float phi_left_nn = WHEEL_POS_SIGN_LEFT * DDSM115MotorList[0].phi_rad;
    float phi_right_nn = WHEEL_POS_SIGN_RIGHT * DDSM115MotorList[1].phi_rad;
    float omega_left_nn = WHEEL_VEL_SIGN_LEFT * DDSM115MotorList[0].phi_dot_rad_s;
    float omega_right_nn = WHEEL_VEL_SIGN_RIGHT * DDSM115MotorList[1].phi_dot_rad_s;
    float yaw_nn_rad = YAW_SIGN * yaw_esp32;

    if (!refs_valid) {
        phi_left_ref_nn = phi_left_nn;
        phi_right_ref_nn = phi_right_nn;
        yaw_ref_nn_rad = yaw_nn_rad;
        refs_valid = true;
    }

    float s_left_m = WHEEL_RADIUS_R * (phi_left_nn - phi_left_ref_nn);
    float s_right_m = WHEEL_RADIUS_R * (phi_right_nn - phi_right_ref_nn);
    float v_left_mps = WHEEL_RADIUS_R * omega_left_nn;
    float v_right_mps = WHEEL_RADIUS_R * omega_right_nn;

    float x_rel_m = 0.5f * (s_left_m + s_right_m);
    float x_dot_mps = 0.5f * (v_left_mps + v_right_mps);
    float theta_rad = PITCH_SIGN * roll_esp32;
    float theta_dot_radps = PITCH_RATE_SIGN * gx_esp32;
    float yaw_error_rad = wrap_pi(yaw_nn_rad - yaw_ref_nn_rad);
    float yaw_rate_radps = YAW_RATE_SIGN * gz_esp32;

    ann_in_data[0] = x_rel_m / NNDRIVE_POS_ERR_NORM_M;                          // pos_err / 0.5m
    ann_in_data[1] = x_dot_mps / NNDRIVE_VEL_CMD_NORM_MPS;                      // velocity / 1.0 m/s
    ann_in_data[2] = theta_rad / 0.43633231f;                                   // pitch / 25deg
    ann_in_data[3] = theta_dot_radps / 4.0f;                                    // pitch_rate / 4 rad/s
    ann_in_data[4] = yaw_error_rad / NNDRIVE_YAW_ERR_NORM_RAD;                  // yaw_err / 1.5 rad
    ann_in_data[5] = yaw_rate_radps / 4.0f;                                     // yaw_rate / 4 rad/s
    ann_in_data[6] = NNDRIVE_VELOCITY_CMD_MPS / NNDRIVE_VEL_CMD_NORM_MPS;       // velocity_cmd (joystick=0)
    ann_in_data[7] = NNDRIVE_YAWRATE_CMD_RPS / NNDRIVE_YAWRATE_CMD_NORM_RPS;    // yaw_rate_cmd (joystick=0)

    // ---- obs 8-11: CyberGear joint angles (SIM frame), order fl/fr/bl/br ----
    // Capture the folded-rest firmware reference once (or on an STM-Studio recapture request),
    // then express every leg angle in sim frame: sim = (.angle - ref) + rest. See NNDRIVE_CG_REST_*.
    if (!cg_ref_captured || CG_REF_recapture) {
        CG_REF_recapture = 0;
        for (int i = 0; i < 4; i++) {
            cg_angle_ref[i] = CyberGearMotorList[i].angle;
        }
        cg_ref_captured = true;
        cg_slew_initialized = false; // force slew re-seed from the freshly-referenced pose
    }

    float cg_sim_angle[4];
    for (int i = 0; i < 4; i++) {
#if NNDRIVE_CG_REST_ENABLE
        cg_sim_angle[i] = cg_dir[i] * (CyberGearMotorList[i].angle - cg_angle_ref[i]) + cg_sim_at_rest[i];
#else
        cg_sim_angle[i] = CyberGearMotorList[i].angle;
#endif
    }

    ann_in_data[8]  = cg_sim_angle[NNDRIVE_CG_IDX_FL] / NNDRIVE_CG_POS_NORM_RAD;
    ann_in_data[9]  = cg_sim_angle[NNDRIVE_CG_IDX_FR] / NNDRIVE_CG_POS_NORM_RAD;
    ann_in_data[10] = cg_sim_angle[NNDRIVE_CG_IDX_BL] / NNDRIVE_CG_POS_NORM_RAD;
    ann_in_data[11] = cg_sim_angle[NNDRIVE_CG_IDX_BR] / NNDRIVE_CG_POS_NORM_RAD;

    // ---- obs 12-13: previous wheel current, left/right ----
    ann_in_data[12] = prev_wheel_current_A[0] / NNDRIVE_PREV_CURRENT_NORM_A;
    ann_in_data[13] = prev_wheel_current_A[1] / NNDRIVE_PREV_CURRENT_NORM_A;

    // ---- obs 14-17: previous CyberGear action, tanh-space, order fl/fr/bl/br ----
    ann_in_data[14] = prev_cg_action[0];
    ann_in_data[15] = prev_cg_action[1];
    ann_in_data[16] = prev_cg_action[2];
    ann_in_data[17] = prev_cg_action[3];

    for (uint32_t i = 0; i < STAI_NETWORK_IN_1_SIZE; i++) {
        if (!isfinite(ann_in_data[i])) {
            ann_in_data[i] = 0.0f;
        } else if (ann_in_data[i] > 10.0f) {
            ann_in_data[i] = 10.0f;
        } else if (ann_in_data[i] < -10.0f) {
            ann_in_data[i] = -10.0f;
        }
    }

    if (stai_network_run(ann_network_context, STAI_MODE_SYNC) != STAI_SUCCESS) {
        ddsm_NN_current_command[0] = 0.0f;
        ddsm_NN_current_command[1] = 0.0f;
        // cg_last_cmd_rad[] left as-is - previous tick's target keeps being sent.
        return;
    }

    bool all_finite = true;
    for (uint32_t i = 0; i < STAI_NETWORK_OUT_1_SIZE; i++) {
        if (!isfinite(ann_out_data[i])) {
            all_finite = false;
            break;
        }
    }
    if (!all_finite) {
        ddsm_NN_current_command[0] = 0.0f;
        ddsm_NN_current_command[1] = 0.0f;
        return;
    }

    // ---- actions 0-3: CyberGear position targets (rad), order fl/fr/bl/br ----
    // Order CONFIRMED against the sim: nn_drive_env.py feeds actions[:, 0:4] straight to
    // _cg_ids, which standup_env.py builds as [front_left, front_right, back_left,
    // back_right]. Modes 1-3 were permutation experiments against the old +M_PI_2
    // command-frame bug (bench lockouts) and are obsolete - keep mode 0.
#define NNDRIVE_ACTION_ORDER_MODE 0
#if NNDRIVE_ACTION_ORDER_MODE == 1
    float raw_bl = ann_out_data[0];
    float raw_br = ann_out_data[1];
    float raw_fl = ann_out_data[2];
    float raw_fr = ann_out_data[3];
#elif NNDRIVE_ACTION_ORDER_MODE == 2
    float raw_br = ann_out_data[0];
    float raw_bl = ann_out_data[1];
    float raw_fr = ann_out_data[2];
    float raw_fl = ann_out_data[3];
#elif NNDRIVE_ACTION_ORDER_MODE == 3
    float raw_fr = ann_out_data[0];
    float raw_fl = ann_out_data[1];
    float raw_br = ann_out_data[2];
    float raw_bl = ann_out_data[3];
#else
    float raw_fl = ann_out_data[0];
    float raw_fr = ann_out_data[1];
    float raw_bl = ann_out_data[2];
    float raw_br = ann_out_data[3];
#endif

    float target_rad[4]; // indexed like CyberGearMotorList[], i.e. physical-array order
    target_rad[NNDRIVE_CG_IDX_FL] = clamp_float(raw_fl, NNDRIVE_CG_FLBR_MIN_RAD, NNDRIVE_CG_FLBR_MAX_RAD);
    target_rad[NNDRIVE_CG_IDX_FR] = clamp_float(raw_fr, NNDRIVE_CG_FRBL_MIN_RAD, NNDRIVE_CG_FRBL_MAX_RAD);
    target_rad[NNDRIVE_CG_IDX_BL] = clamp_float(raw_bl, NNDRIVE_CG_FRBL_MIN_RAD, NNDRIVE_CG_FRBL_MAX_RAD);
    target_rad[NNDRIVE_CG_IDX_BR] = clamp_float(raw_br, NNDRIVE_CG_FLBR_MIN_RAD, NNDRIVE_CG_FLBR_MAX_RAD);

    // First valid tick since a (re)start: seed the (sim-frame) slew memory from the current
    // sim pose so the first commanded angle doesn't jump. On the very first capture cg_sim_angle
    // equals the folded-rest value; after a fall-recovery it reflects wherever the legs are now.
    if (!cg_slew_initialized) {
        for (int i = 0; i < 4; i++) {
            cg_last_cmd_rad[i] = cg_sim_angle[i];
        }
        cg_slew_initialized = true;
    }

    float max_step_rad = NNDRIVE_CG_SLEW_RATE_RADPS * NNDRIVE_TICK_DT_S; // 3.0 * 0.015 = 0.045 rad/tick
    for (int i = 0; i < 4; i++) {
        float delta = target_rad[i] - cg_last_cmd_rad[i];
        if (delta > max_step_rad) delta = max_step_rad;
        if (delta < -max_step_rad) delta = -max_step_rad;
        cg_last_cmd_rad[i] += delta;
    }

    // obs[14-17] in the sim are the RAW tanh action, captured BEFORE the stance processor
    // clamps and slews (nn_drive_env.py stores CyberGearStanceProcessor.tanh_action). The
    // exported ONNX already outputs tanh(a)*CG_ACTION_SCALE_RAD, so dividing the raw network
    // output recovers the exact tanh-space value.
    prev_cg_action[0] = clamp_float(raw_fl / CG_ACTION_SCALE_RAD, -1.0f, 1.0f);
    prev_cg_action[1] = clamp_float(raw_fr / CG_ACTION_SCALE_RAD, -1.0f, 1.0f);
    prev_cg_action[2] = clamp_float(raw_bl / CG_ACTION_SCALE_RAD, -1.0f, 1.0f);
    prev_cg_action[3] = clamp_float(raw_br / CG_ACTION_SCALE_RAD, -1.0f, 1.0f);

    // Convert the sim-frame slew target back to firmware frame for the desired_angle write:
    // firmware = ref + dir*(sim - rest). The main loop writes cg_cmd_fw[] into CyberGearMotorList[].desired_angle.
    for (int i = 0; i < 4; i++) {
#if NNDRIVE_CG_REST_ENABLE
        cg_cmd_fw[i] = cg_angle_ref[i] + cg_dir[i] * (cg_last_cmd_rad[i] - cg_sim_at_rest[i]);
#else
        cg_cmd_fw[i] = cg_last_cmd_rad[i];
#endif
    }

    // ---- actions 4-5: wheel currents (A) ----
    float left_nn_current_A = ann_out_data[4];
    float right_nn_current_A = ann_out_data[5];

    prev_wheel_current_A[0] = left_nn_current_A;
    prev_wheel_current_A[1] = right_nn_current_A;

    ddsm_NN_current_command[0] = clamp_float(DDSM_CURRENT_SIGN_LEFT * left_nn_current_A,
                                             -STM32_TEST_CURRENT_LIMIT_A,
                                             STM32_TEST_CURRENT_LIMIT_A);
    ddsm_NN_current_command[1] = clamp_float(DDSM_CURRENT_SIGN_RIGHT * right_nn_current_A,
                                             -STM32_TEST_CURRENT_LIMIT_A,
                                             STM32_TEST_CURRENT_LIMIT_A);
}
#endif // NN_DRIVE_POLICY_ACTIVE

#if CYBERGEAR_INDEX_BENCH_TEST
// One-shot, non-blocking bench routine to identify which CyberGearMotorList[] index
// drives which physical corner. Robot must be hand-held or resting unloaded on a stand
// (NOT standing on its own wheels) while this runs. Write CG_BENCH_start = 1 to begin;
// it sweeps index 0->3, nudging each one at a time by CG_BENCH_STEP_RAD and back, with
// CG_BENCH_active_index showing which index is currently being tested so you can watch
// the robot and note which corner moves. Call this once per 15ms tick in place of the
// normal NN-drive control while it's active.
#define CG_BENCH_STEP_RAD (0.12f)  // ~6.9deg - safe/visible under any possible corner's real limits
#define CG_BENCH_HOLD_S    (2.0f)  // time spent at the nudged-out angle, and again back at start
#define CG_BENCH_PAUSE_S   (1.5f)  // pause between motors so the transition is easy to see

typedef enum {
    CG_BENCH_PHASE_IDLE = 0,
    CG_BENCH_PHASE_OUT,
    CG_BENCH_PHASE_BACK,
    CG_BENCH_PHASE_PAUSE,
} CG_BenchPhase;

static void CyberGear_IndexBenchTest(void)
{
    static CG_BenchPhase phase = CG_BENCH_PHASE_IDLE;
    static float phase_timer_s = 0.0f;
    static float base_angle_rad = 0.0f;
    static int   index = 0;

    if (CG_BENCH_start && phase == CG_BENCH_PHASE_IDLE) {
        CG_BENCH_start = 0;
        // Safety: make sure the wheels aren't still driving from a previous run.
        DDSM115setCurrent(0x10, 0);
        DDSM115setCurrent(0x11, 0);
        index = 0;
        base_angle_rad = CyberGearMotorList[index].angle;
        CG_BENCH_active_index = index;
        phase = CG_BENCH_PHASE_OUT;
        phase_timer_s = 0.0f;
    }

    if (phase == CG_BENCH_PHASE_IDLE) {
        return;
    }

    phase_timer_s += NNDRIVE_TICK_DT_S;

    float target = (phase == CG_BENCH_PHASE_OUT) ? (base_angle_rad + CG_BENCH_STEP_RAD) : base_angle_rad;

    if (phase != CG_BENCH_PHASE_PAUSE) {
        CyberGearMotorList[index].desired_angle = target;
        Motor_SendMITCommand(&CyberGearMotorList[index]);
    }

    switch (phase) {
        case CG_BENCH_PHASE_OUT:
            if (phase_timer_s >= CG_BENCH_HOLD_S) { phase = CG_BENCH_PHASE_BACK; phase_timer_s = 0.0f; }
            break;
        case CG_BENCH_PHASE_BACK:
            if (phase_timer_s >= CG_BENCH_HOLD_S) { phase = CG_BENCH_PHASE_PAUSE; phase_timer_s = 0.0f; }
            break;
        case CG_BENCH_PHASE_PAUSE:
            if (phase_timer_s >= CG_BENCH_PAUSE_S) {
                index++;
                if (index >= 4) {
                    phase = CG_BENCH_PHASE_IDLE;
                    CG_BENCH_active_index = -1;
                } else {
                    base_angle_rad = CyberGearMotorList[index].angle;
                    CG_BENCH_active_index = index;
                    phase = CG_BENCH_PHASE_OUT;
                    phase_timer_s = 0.0f;
                }
            }
            break;
        default:
            break;
    }
}
#endif // CYBERGEAR_INDEX_BENCH_TEST

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

    float theta_deg = (PITCH_SIGN * roll_esp32) * MET_RAD2DEG;
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

    float i_avg_A = 0.5f * (fabsf(ddsm_NN_current_command[0]) + fabsf(ddsm_NN_current_command[1]));

    sample_count++;
    sum_theta_sq += theta_deg * theta_deg;
    sum_I_sq     += i_avg_A * i_avg_A;
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
// after ANN_Run() so ann_in_data/ann_out_data/ddsm_NN_current_command reflect this
// cycle's values. See the LOG_ globals above for the timeseries_log_template.csv
// column mapping.
static void LOG_Update(void)
{
    LOG_theta_deg = (PITCH_SIGN * roll_esp32) * MET_RAD2DEG;

    for (uint32_t i = 0; i < STAI_NETWORK_IN_1_SIZE; i++) {
        LOG_obs[i] = ann_in_data[i];
    }

#if NN_DRIVE_POLICY_ACTIVE
    LOG_action_cg_fl = ann_out_data[0];
    LOG_action_cg_fr = ann_out_data[1];
    LOG_action_cg_bl = ann_out_data[2];
    LOG_action_cg_br = ann_out_data[3];
    LOG_action_L = ann_out_data[4];
    LOG_action_R = ann_out_data[5];
#else
    LOG_action_L = ann_out_data[0];
    LOG_action_R = ann_out_data[1];
#endif

    LOG_I_L_cmd_A = ddsm_NN_current_command[0];
    LOG_I_R_cmd_A = ddsm_NN_current_command[1];

    // No current feedback from the DDSM115 drives - mirror the commanded value.
    LOG_I_L_meas_A = ddsm_NN_current_command[0];
    LOG_I_R_meas_A = ddsm_NN_current_command[1];
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
