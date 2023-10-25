#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "ch.h"
#include "hal.h"
#include "memory_protection.h"
//#include "sensors/proximity.h"
#include <main.h>

//LEDS
//#include "leds.h"
#include "spi_comm.h"

//MOTORS
#include "motors.h"

messagebus_t bus;
MUTEX_DECL(bus_lock);
CONDVAR_DECL(bus_condvar);

void avoid_collisions(unsigned int noiseLevel, unsigned int activationThreshold, int inputLeftVelocity, int inputRightVelocity) {
	int i;
	int outputLeftVelocity = inputLeftVelocity;
	int outputRightVelocity = inputRightVelocity;
	int* outputRightVelocityPtr = &outputRightVelocity;
	int* outputLeftVelocityPtr = &outputLeftVelocity;
	unsigned int sensorValues[8];
	double xWeights[] = {-1, -0.7, 0, 0.9, 0.9, 0, -0.7, -1};
	double yWeights[] = {0.3, 0.7, 1, 0.5, -0.5, -1, -0.7, -0.3};
	int xSensorTotal = 0;
	int ySensorTotal = 0;

	// Remove noise and readings too far away.
	for(i = 0; i < 8; i++) {
		sensorValues[i] = get_calibrated_prox(i);
		if(sensorValues[i] >= noiseLevel && sensorValues[i] >= activationThreshold) {
			xSensorTotal += xWeights[i] * sensorValues[i];
			ySensorTotal += yWeights[i] * sensorValues[i];
		}
	}

	// Modify the velocity components based on sensor values.
	if(inputLeftVelocity >= 0) {
		outputLeftVelocity += ((xSensorTotal * 0.8) - (ySensorTotal * 0.5));
	} else {
		outputLeftVelocity -= ((xSensorTotal * 0.8) + (ySensorTotal * 0.5));
	}
	if(inputRightVelocity >= 0) {
		outputRightVelocity += ((xSensorTotal * 0.8) + (ySensorTotal * 0.5));
	} else {
		outputRightVelocity -= ((xSensorTotal * 0.8) - (ySensorTotal * 0.5));
	}



	// Update speed.
	left_motor_set_speed(outputLeftVelocity);
	right_motor_set_speed(outputRightVelocity);
}

void check_stationary() {
	if outputRightVelocityPtr = 0;
}

int main(void)
{

    halInit();
    chSysInit();
    mpu_init();

    spi_comm_start();

    // prox sensors
    messagebus_init(&bus, &bus_lock, &bus_condvar);
    proximity_start();
    calibrate_ir();

//    MOTORS
    motors_init();


    //move forward
	int motorForward = 1000;
    left_motor_set_speed(motorForward);
    right_motor_set_speed(motorForward);

    /* Infinite loop. continuously avoid obstacles */
    while (1) {
    	avoid_collisions(5, 150, motorForward, motorForward);
    	chThdSleepMilliseconds(100);



    }
}

#define STACK_CHK_GUARD 0xe2dee396
uintptr_t __stack_chk_guard = STACK_CHK_GUARD;

void __stack_chk_fail(void)
{
    chSysHalt("Stack smashing detected");
}
