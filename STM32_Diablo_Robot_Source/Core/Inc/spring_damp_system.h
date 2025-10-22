/*
 * spring_damp_system.h
 *
 *  Created on: Mar 8, 2025
 *      Author: tofan
 */

#ifndef INC_SPRING_DAMP_SYSTEM_H_
#define INC_SPRING_DAMP_SYSTEM_H_

// Maximum number of spring-damp systems.
#define MAX_SPRING_DAMP_SYSTEMS 4

// Define a structure to hold the parameters of a spring-damp system.

typedef struct {
	float mass;     // Mass of the object
	float k;        // Spring constant
	float b;        // Damping coefficient
	float l;        // Position of the object
	float l_dot;        // Velocity of the object
	float l_eq;        // Equilibrium position
} SpringDampSystem;

// Extern declaration of the spring-damp system.
extern SpringDampSystem SpringDampSystemList[MAX_SPRING_DAMP_SYSTEMS];


#endif /* INC_SPRING_DAMP_SYSTEM_H_ */
