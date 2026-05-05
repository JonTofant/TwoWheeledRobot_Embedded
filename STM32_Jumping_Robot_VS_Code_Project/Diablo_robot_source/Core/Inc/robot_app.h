/*
 * robot_app.h
 *
 * Hand-written application layer kept outside CubeMX generated sections.
 */

#ifndef INC_ROBOT_APP_H_
#define INC_ROBOT_APP_H_

#include "main.h"

void Robot_AppStart(void);
void Robot_AppRunOnce(void);

void Robot_AppCanRxFifo0Pending(CAN_HandleTypeDef *hcan);
void Robot_AppUartRxComplete(UART_HandleTypeDef *huart);
void Robot_AppUartRxEvent(UART_HandleTypeDef *huart, uint16_t size);
void Robot_AppTimerElapsed(TIM_HandleTypeDef *htim);
void Robot_AppUartTxComplete(UART_HandleTypeDef *huart);

#endif /* INC_ROBOT_APP_H_ */
