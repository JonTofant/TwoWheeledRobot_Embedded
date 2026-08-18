/*
 * system_init.c
 *
 *  Created on: Apr 13, 2025
 *      Author: tofan
 */


#include "system_init.h"
#include "cybergear.h"

// MOTOR_CG_LF/LB/RF/RB come from DDSM115.h (included via system_init.h). Do not
// redefine them here - a previous local redefinition had LF/LB swapped relative to
// that canonical mapping, which telemetry.c and controler.c both rely on, causing
// System_Init()'s motorEnable/setMechanicalZero/writeParameter/Motor_SendAngle calls
// to act on the wrong physical leg motor for LF/LB.
// Function that initializes the system
void System_Init(void)
{
	// Set the cybergear motor to angle mode
	  // 2) Enable motor
	  motorEnable(MOTOR_CG_LF.hostID, MOTOR_CG_LF.motorID);
	  HAL_Delay(50);

	  motorEnable(MOTOR_CG_LB.hostID, MOTOR_CG_LB.motorID);

	  HAL_Delay(50);

	  motorEnable(MOTOR_CG_RF.hostID, MOTOR_CG_RF.motorID);
	  HAL_Delay(50);

	  motorEnable(MOTOR_CG_RB.hostID, MOTOR_CG_RB.motorID);
	  HAL_Delay(50);



	  // Zero the motor
	  setMechanicalZero(MOTOR_CG_LF.hostID, MOTOR_CG_LF.motorID);
	  HAL_Delay(50);

	  setMechanicalZero(MOTOR_CG_LB.hostID, MOTOR_CG_LB.motorID);
	  HAL_Delay(50);

	  setMechanicalZero(MOTOR_CG_RF.hostID, MOTOR_CG_RF.motorID);
	  HAL_Delay(50);

	  setMechanicalZero(MOTOR_CG_RB.hostID, MOTOR_CG_RB.motorID);
	  HAL_Delay(50);

	  // DDSM115 change ID
	  //DDSM115ChangeID(0xAA, 0x10); // Change ID of first motor to 0x10

	  //2) Put motor in position mode
	  	  // TODO change this into a function
	  	  /*uint8_t runMode = 1; // 1 => position mode, 3 => current mode
	  	  writeParameter(0x7005, &runMode, MOTOR_CG_LF.hostID, MOTOR_CG_LF.motorID);
	  	  HAL_Delay(50);
	  	  writeParameter(0x7005, &runMode, MOTOR_CG_LB.hostID, MOTOR_CG_LB.motorID);
	  	  HAL_Delay(50);
	  	  writeParameter(0x7005, &runMode, MOTOR_CG_RF.hostID, MOTOR_CG_RF.motorID);
	  	  HAL_Delay(50);
	  	  writeParameter(0x7005, &runMode, MOTOR_CG_RB.hostID, MOTOR_CG_RB.motorID);
	  	  HAL_Delay(50);

	  	  writeParameter(0x7017, &MOTOR_CG_LF.max_velocity, MOTOR_CG_LF.hostID, MOTOR_CG_LF.motorID);
	  	  HAL_Delay(50);

	  	  writeParameter(0x7017, &MOTOR_CG_LB.max_velocity, MOTOR_CG_LB.hostID, MOTOR_CG_LB.motorID);
	  	  HAL_Delay(50);

	  	  writeParameter(0x7017, &MOTOR_CG_RF.max_velocity, MOTOR_CG_RF.hostID, MOTOR_CG_RF.motorID);
	  	  HAL_Delay(50);

	  	  writeParameter(0x7017, &MOTOR_CG_RB.max_velocity, MOTOR_CG_RB.hostID, MOTOR_CG_RB.motorID);
	  	  HAL_Delay(50);
*/


	  	  // ------------------DEMO INICIALIZACIJA----------------

	  	  // Set the angle for both for one motor to 50deg and the other to -50deg

	  	  // Decide target angle for each motor
	  	/*  MOTOR_CG_LF.target_angle = 0.785398163f;

	  	  MOTOR_CG_LB.target_angle = -0.785398163f;

	  	  MOTOR_CG_RF.target_angle = -0.785398163f;

	  	  MOTOR_CG_RB.target_angle = 0.785398163f;*/

	  	  // DEBUG MODE
	  	  /*
	  	  MOTOR_CG_LF.target_angle = 0.523598776f;

	  	  MOTOR_CG_LB.target_angle = -0.523598776f;

	  	  MOTOR_CG_RF.target_angle = -0.523598776f;

	  	  MOTOR_CG_RB.target_angle = 0.523598776f;
	  	  */
	  	  MOTOR_CG_LF.target_angle = 0.0;

	  	  MOTOR_CG_LB.target_angle = -0.0;

	  	  MOTOR_CG_RF.target_angle = -0.0;

	  	  MOTOR_CG_RB.target_angle = 0.0;


	  	  // Send the target angle to each motor
	  	//Motor_SendMITCommand(&MOTOR_CG_LF, 1.0);
	  	//HAL_Delay(70);
	  	//Motor_SendMITCommand(&MOTOR_CG_LB, 1.0);
	  	//HAL_Delay(70);
	  	//Motor_SendMITCommand(&MOTOR_CG_RF, 1.0);
	  	//HAL_Delay(70);
	  	//Motor_SendMITCommand(&MOTOR_CG_RB, 1.0);
	  	//HAL_Delay(70);

	  	/*
	  	  Motor_SendAngle(&MOTOR_CG_LF);
	  	  HAL_Delay(70);
	  	  Motor_SendAngle(&MOTOR_CG_LB);
	  	  HAL_Delay(70);
	  	  Motor_SendAngle(&MOTOR_CG_RF);
	  	  HAL_Delay(70);
	  	  Motor_SendAngle(&MOTOR_CG_RB);
	  	  HAL_Delay(70);
*/
	  	  /* Ta Mechanical zero ne dela nic
		  // Zero the motor
		  setMechanicalZero(MOTOR_CG_LF.hostID, MOTOR_CG_LF.motorID);
		  HAL_Delay(50);

		  setMechanicalZero(MOTOR_CG_LB.hostID, MOTOR_CG_LB.motorID);
		  HAL_Delay(50);

		  setMechanicalZero(MOTOR_CG_RF.hostID, MOTOR_CG_RF.motorID);
		  HAL_Delay(50);

		  setMechanicalZero(MOTOR_CG_RB.hostID, MOTOR_CG_RB.motorID);
		  HAL_Delay(50);

		  HAL_Delay(3000);*/

	/* Use the same addressed status/mode/zero-current transactions as the
	 * known-good DDSM115 characterization firmware. Startup results remain in
	 * DDSM115_startup_* for inspection in STM Studio/the debugger. */
	(void)DDSM115InitializeCurrentMode();

}
