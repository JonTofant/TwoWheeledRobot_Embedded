/*
 * system_init.c
 *
 *  Created on: Apr 13, 2025
 *      Author: tofan
 */

#include "system_init.h"

#include "controler.h"
#include "cybergear.h"

#define STARTUP_MOTOR_CG_LB CyberGearMotorList[0]
#define STARTUP_MOTOR_CG_LF CyberGearMotorList[1]
#define STARTUP_MOTOR_CG_RF CyberGearMotorList[2]
#define STARTUP_MOTOR_CG_RB CyberGearMotorList[3]

static void EnableCybergearMotor(CyberGear *motor);
static void ZeroCybergearMotor(CyberGear *motor);
static void SetInitialCybergearTargets(void);
static void SetDriveMotorsToCurrentMode(void);

void System_Init(void)
{
    EnableCybergearMotor(&STARTUP_MOTOR_CG_LF);
    EnableCybergearMotor(&STARTUP_MOTOR_CG_LB);
    EnableCybergearMotor(&STARTUP_MOTOR_CG_RF);
    EnableCybergearMotor(&STARTUP_MOTOR_CG_RB);

    ZeroCybergearMotor(&STARTUP_MOTOR_CG_LF);
    ZeroCybergearMotor(&STARTUP_MOTOR_CG_LB);
    ZeroCybergearMotor(&STARTUP_MOTOR_CG_RF);
    ZeroCybergearMotor(&STARTUP_MOTOR_CG_RB);

    SetInitialCybergearTargets();
    SetDriveMotorsToCurrentMode();
    MIT_controler_gain_schedule_Normal();
}

static void EnableCybergearMotor(CyberGear *motor)
{
    motorEnable(motor->hostID, motor->motorID);
    HAL_Delay(50);
}

static void ZeroCybergearMotor(CyberGear *motor)
{
    setMechanicalZero(motor->hostID, motor->motorID);
    HAL_Delay(50);
}

static void SetInitialCybergearTargets(void)
{
    STARTUP_MOTOR_CG_LF.target_angle = 0.0f;
    STARTUP_MOTOR_CG_LB.target_angle = -0.0f;
    STARTUP_MOTOR_CG_RF.target_angle = -0.0f;
    STARTUP_MOTOR_CG_RB.target_angle = 0.0f;
}

static void SetDriveMotorsToCurrentMode(void)
{
    DDMS115setMode(0x10, 0x01);
    HAL_Delay(10);
    DDMS115setMode(0x11, 0x01);
}
