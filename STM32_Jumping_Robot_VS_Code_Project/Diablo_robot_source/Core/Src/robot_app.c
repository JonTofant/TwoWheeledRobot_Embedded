/*
 * robot_app.c
 *
 * Application startup, scheduler flags, and peripheral callbacks.
 */

#include "robot_app.h"

#include <math.h>
#include <stdbool.h>
#include <string.h>

#include "DDSM115.h"
#include "JumpStrategy.h"
#include "StartupStrategy.h"
#include "StateEstimator.h"
#include "controler.h"
#include "cybergear.h"
#include "joystick.h"
#include "kinematics.h"
#include "state_machine.h"
#include "system_init.h"
#include "telemetry.h"

#define ESP32_IMU_PACKET_SIZE 19U
#define UART1_RX_BUFFER_SIZE 128U
#define DDSM115_SAMPLE_WHEEL_RADIUS_M 0.0505f

extern CAN_HandleTypeDef hcan1;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart3;
extern UART_HandleTypeDef huart5;

static CAN_RxHeaderTypeDef can_rx_header;
static uint8_t rs485_rx_buffer[RS485_BUFFER_SIZE];
static uint8_t uart1_rx_buffer[UART1_RX_BUFFER_SIZE];

static void StartEsp32ImuReception(void);
static void StartDdsm115Reception(void);
static void StartJoystickReception(void);
static void EnableCycleCounter(void);
static void QueryDdsm115(void);
static void ConfigureCanReception(void);
static void InitializeLegStates(void);
static void StartControlTimers(void);
static void ProcessJoystickPacket(void);
static void ProcessCybergearTask(void);
static void ProcessDdsm115Task(void);
static void SendLegMitCommands(void);
static void HandleCybergearFeedback(void);
static void HandleDdsm115Packet(void);
static void HandleEsp32ImuPacket(uint16_t size);

void Robot_AppStart(void)
{
    StartEsp32ImuReception();
    StartDdsm115Reception();
    EnableCycleCounter();
    QueryDdsm115();
    ConfigureCanReception();

    HAL_Delay(3000);

    System_Init();
    InitializeLegStates();
    StartControlTimers();
    StartJoystickReception();
}

void Robot_AppRunOnce(void)
{
    if (isCANReady) {
        isCANReady = false;
    }

    ProcessJoystickPacket();
    ProcessCybergearTask();
    ProcessDdsm115Task();

    if (isTELEMETRYReady) {
        Send_Telemetry(&huart3);
    }
}

void Robot_AppCanRxFifo0Pending(CAN_HandleTypeDef *hcan)
{
    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &can_rx_header, CAN_received_data) != HAL_OK) {
        return;
    }

    HandleCybergearFeedback();
}

void Robot_AppUartRxComplete(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART3) {
        if (uart3_controller_index == 0U) {
            if (uart3_controller_byte == 0xAAU) {
                uart3_controller_buf[uart3_controller_index++] = uart3_controller_byte;
            }
        } else {
            uart3_controller_buf[uart3_controller_index++] = uart3_controller_byte;

            if (uart3_controller_index >= UART3_CONTROLLER_PACKET_LEN) {
                uart3_controller_index = 0U;
                uart3_controller_packet_ready = 1U;
            }
        }

        HAL_UART_Receive_IT(&huart3, &uart3_controller_byte, 1);
    }
}

void Robot_AppUartRxEvent(UART_HandleTypeDef *huart, uint16_t size)
{
    if (huart->Instance == UART5) {
        HandleDdsm115Packet();
        StartDdsm115Reception();
    }

    if (huart->Instance == USART1) {
        HandleEsp32ImuPacket(size);
        StartEsp32ImuReception();
    }
}

void Robot_AppTimerElapsed(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM4) {
        isDDSM115Ready = true;
        isCYBERGEARReady = true;
        isTELEMETRYReady = true;
    }
}

void Robot_AppUartTxComplete(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART3) {
        Telemetry_UART_TxCpltCallback(huart);
    }
}

static void StartEsp32ImuReception(void)
{
    HAL_UARTEx_ReceiveToIdle_IT(&huart1, uart1_rx_buffer, UART1_RX_BUFFER_SIZE);
}

static void StartDdsm115Reception(void)
{
    HAL_UARTEx_ReceiveToIdle_IT(&huart5, rs485_rx_buffer, RS485_BUFFER_SIZE);
}

static void StartJoystickReception(void)
{
    HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);
    HAL_UART_Receive_IT(&huart3, &uart3_controller_byte, 1);
}

static void EnableCycleCounter(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

static void QueryDdsm115(void)
{
    uint8_t cmd[RS485_BUFFER_SIZE] = {
        0xAA, 0x64, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00
    };

    cmd[9] = compute_crc8(cmd, 9);

    HAL_GPIO_WritePin(RS485_DIR_GPIO_Port, RS485_DIR_Pin, GPIO_PIN_SET);
    HAL_UART_Transmit(&huart5, cmd, sizeof(cmd), HAL_MAX_DELAY);
    HAL_Delay(10);
    HAL_GPIO_WritePin(RS485_DIR_GPIO_Port, RS485_DIR_Pin, GPIO_PIN_RESET);
}

static void ConfigureCanReception(void)
{
    CAN_FilterTypeDef filter = {
        .FilterBank = 0,
        .FilterMode = CAN_FILTERMODE_IDMASK,
        .FilterScale = CAN_FILTERSCALE_32BIT,
        .FilterIdHigh = 0x0000,
        .FilterIdLow = 0x0000,
        .FilterMaskIdHigh = 0x0000,
        .FilterMaskIdLow = 0x0000,
        .FilterFIFOAssignment = CAN_FILTER_FIFO0,
        .FilterActivation = ENABLE,
        .SlaveStartFilterBank = 14
    };

    HAL_CAN_ConfigFilter(&hcan1, &filter);
    HAL_CAN_Start(&hcan1);
    HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);

    HAL_NVIC_SetPriority(CAN1_RX0_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(CAN1_RX0_IRQn);
}

static void InitializeLegStates(void)
{
    init_leg_state(&leg_state_rf);
    init_leg_state(&leg_state_lf);
    init_leg_state(&leg_state_rb);
    init_leg_state(&leg_state_lb);
}

static void StartControlTimers(void)
{
    HAL_TIM_Base_Start_IT(&htim3);
    HAL_TIM_Base_Start_IT(&htim4);
}

static void ProcessJoystickPacket(void)
{
    if (uart3_controller_packet_ready) {
        process_joystick_input();
        uart3_controller_packet_ready = 0U;
    }
}

static void ProcessCybergearTask(void)
{
    if (!isCYBERGEARReady) {
        return;
    }

    if (isFallen()) {
        if (startPressed == 1U) {
            isStartupStrategy = true;
        }
    } else if (xPressed == 1U) {
        isJumpStrategy = true;
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

    if (!isFallen()) {
        if (isSTATIC) {
            posture_controler();
            SendLegMitCommands();
            isCYBERGEARReady = false;
        } else if (isLOCOMOTION) {
            posture_controler();
            SendLegMitCommands();
            isCYBERGEARReady = false;
        }
    } else {
        isCYBERGEARReady = false;
    }
}

static void ProcessDdsm115Task(void)
{
    if (!isDDSM115Ready) {
        return;
    }

    if (!isFallen() || isStartupStrategy) {
        calculate_cascaded_motor_currents(desired_v_left,
                                          desired_v_right,
                                          &current_motor1_out,
                                          &current_motor2_out,
                                          &total_force_out);

        DDSM115setCurrent(0x10, current_motor2_out);
        HAL_Delay(2);
        DDSM115setCurrent(0x11, current_motor1_out);
    }

    isDDSM115Ready = false;
}

static void SendLegMitCommands(void)
{
    Motor_SendMITCommand(&MOTOR_CG_LF);
    Motor_SendMITCommand(&MOTOR_CG_LB);
    HAL_Delay(5);
    Motor_SendMITCommand(&MOTOR_CG_RF);
    Motor_SendMITCommand(&MOTOR_CG_RB);
}

static void HandleCybergearFeedback(void)
{
    uint32_t ext_id = can_rx_header.ExtId;
    uint8_t type = (uint8_t)((ext_id >> 24) & 0x1FU);
    uint8_t motor_id = (uint8_t)((ext_id >> 8) & 0xFFFFU);

    if (type != 2U) {
        return;
    }

    uint16_t angle_raw = (uint16_t)((CAN_received_data[0] << 8) | CAN_received_data[1]);
    uint16_t velocity_raw = (uint16_t)((CAN_received_data[2] << 8) | CAN_received_data[3]);
    uint16_t torque_raw = (uint16_t)((CAN_received_data[4] << 8) | CAN_received_data[5]);
    uint16_t temp_raw = (uint16_t)((CAN_received_data[6] << 8) | CAN_received_data[7]);

    float angle = ((float)angle_raw / 65535.0f) * (ANGLE_MAX - ANGLE_MIN)
                + ANGLE_MIN + M_PI_2 - 10.0f * M_PI / 180.0f;
    float velocity = ((float)velocity_raw / 65535.0f) * (-30.0f - 30.0f) + 30.0f;
    float torque = ((float)torque_raw / 65535.0f) * (-12.0f - 12.0f) + 12.0f;
    float temperature = (float)temp_raw / 10.0f;

    for (int i = 0; i < MAX_MOTORS; i++) {
        if (CyberGearMotorList[i].motorID == motor_id) {
            CyberGearMotorList[i].angle = angle;
            CyberGearMotorList[i].velocity = velocity;
            CyberGearMotorList[i].torque = torque;
            CyberGearMotorList[i].temperature = temperature;
            CyberGearMotorList[i].update_flag = true;
            break;
        }
    }
}

static void HandleDdsm115Packet(void)
{
    uint8_t motor_id = rs485_rx_buffer[0];

    for (int i = 0; i < MAX_MOTORS_DDSM115; i++) {
        if (DDSM115MotorList[i].motorID == motor_id) {
            update_ddsm115_state(&DDSM115MotorList[i],
                                 rs485_rx_buffer,
                                 DDSM115_SAMPLE_WHEEL_RADIUS_M);
            break;
        }
    }
}

static void HandleEsp32ImuPacket(uint16_t size)
{
    uint16_t idx = 0;

    while (idx + ESP32_IMU_PACKET_SIZE <= size) {
        if (uart1_rx_buffer[idx] == 0xAAU) {
            if (!uartSynced) {
                uartSynced = true;
                memmove(uart1_rx_buffer,
                        uart1_rx_buffer + idx,
                        ESP32_IMU_PACKET_SIZE);
                idx = 0;
                size = ESP32_IMU_PACKET_SIZE;
            }

            float *imu_values = (float *)(uart1_rx_buffer + idx + 1U);
            yaw_esp32 = imu_values[0];
            pitch_esp32 = imu_values[1];
            roll_esp32 = imu_values[2];
            gx_esp32 = imu_values[3];
            gy_esp32 = imu_values[4];
            gz_esp32 = imu_values[5];

            idx += ESP32_IMU_PACKET_SIZE;
        } else {
            idx++;
        }
    }
}
