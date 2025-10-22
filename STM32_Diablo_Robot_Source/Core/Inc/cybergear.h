/*
 * cybergear.h
 *
 *  Created on: Mar 5, 2025
 *      Author: tofant
 */

#ifndef INC_CYBERGEAR_H_
#define INC_CYBERGEAR_H_

#include "stm32f4xx_hal.h"  // This includes the HAL definitions including HAL_StatusTypeDef.
#include <stdbool.h>
#include <stdint.h>


#define MAX_MOTORS 4
#define ANGLE_MIN  -12.5f
#define ANGLE_MAX   12.5f


// Motor structure definition with an error flag.
typedef struct {
    uint8_t hostID;
    uint8_t motorID;
    float angle;       // Current target angle (radians)
    float min_angle;   // Minimum allowed angle
    float max_angle;   // Maximum allowed angle
    float kp;          // Position control gain
    float kd;          // Derivative gain
    bool errorFlag;    // True if error or no valid response
    float velocity;    // Current velocity
    float max_velocity; // Maximum velocity
    float torque;      // Current torque
    float temperature; // Current temperature
    float target_angle; // Target angle
    float target_current; // Target current
    float toque_constant; // Torque constant
    float update_flag; // Update flag for kinematics
    float target_current_LQR; // Target current for LQR
} CyberGear;


// Extern declaration of motor array.
extern CyberGear CyberGearMotorList[MAX_MOTORS];

// Function prototypes for motor-related operations.
void Motor_SendAngle(CyberGear* motor);
void Motor_RequestDeviceIDs(void);
HAL_StatusTypeDef clearMotorFault(uint8_t hostID, uint8_t motorID);
HAL_StatusTypeDef motorEnable(uint8_t hostID, uint8_t motorID);
HAL_StatusTypeDef writeParameter(uint16_t paramIndex, const volatile void* paramValue,
                                 uint8_t hostID, uint8_t motorID);
HAL_StatusTypeDef singleParameterRead(uint16_t paramIndex, uint8_t hostID, uint8_t motorID);
HAL_StatusTypeDef getMotorDeviceID(uint8_t hostID, uint8_t motorID);

HAL_StatusTypeDef setMechanicalZero(uint8_t hostID, uint8_t motorID);
uint16_t float_to_uint(float x, float x_min, float x_max);
// Function for setIqRef()
HAL_StatusTypeDef setIqRef(CyberGear* motor, float current);
// Function for motor stop
HAL_StatusTypeDef motorStop(CyberGear* motor);





#endif /* INC_CYBERGEAR_H_ */
