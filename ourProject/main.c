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

#define ROTATE_MODE 1
#define NAVIGATE_MODE 2


int mode;
int mode = NAVIGATE_MODE;
int outputRightVelocity;
int outputLeftVelocity;
int* outputRightVelocityPtr = &outputRightVelocity;
int* outputLeftVelocityPtr = &outputLeftVelocity;


void avoid_collisions(unsigned int noiseLevel, unsigned int activationThreshold, int inputLeftVelocity, int inputRightVelocity) {
	/* avoid collisions using a vector sum calculation from all 8 proximaty sensors. */

	int i;
	outputLeftVelocity = inputLeftVelocity;
	outputRightVelocity = inputRightVelocity;
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
	/* Function records into a buffer the last 10 wheel speeds
	 * for both wheels. Then if below a threshold, will change
	 * from NAVIGATION_MODE to ROTATE_MODE. */
	unsigned int lastRotation = 11;
    unsigned int i2 = 0;
    unsigned int sum;
    unsigned int currentSpeeds[10][2]; // 2D array for circular buffer

    // Get currentSpeedValue
    int currentSpeedRight = abs(*outputRightVelocityPtr);
    int currentSpeedLeft = abs(*outputLeftVelocityPtr);


    // Update circular buffer.
    currentSpeeds[i2][0] = currentSpeedLeft;
    currentSpeeds[i2][1] = currentSpeedRight;

    sum = 0;
    for (int i = 0; i < 10; ++i) {
        for (int j = 0; j < 2; ++j) {
            sum += currentSpeeds[i][j];
        }
    }

    // if stationary, rotate.
    if (sum < 400 && lastRotation != i2 ) {
    	mode = ROTATE_MODE;
    	lastRotation = i2;
    }

    // Update index i (circular manner)
	i2 = i2++;
	i2 % 10;

    return;
}

void rotate(int speed, int time) {

	left_motor_set_speed(speed);
	right_motor_set_speed(-speed);

	chThdSleepMilliseconds(time);

	left_motor_set_speed(0);
	right_motor_set_speed(0);

	mode = NAVIGATE_MODE;

}



int main(void)
{
	//LITERALLY CAN'T REMEMBER WHAT THESE DO
    halInit();
    chSysInit();
    mpu_init();
    //LEDS?
    spi_comm_start();
    // PROX SENSORS
    messagebus_init(&bus, &bus_lock, &bus_condvar);
    proximity_start();
    calibrate_ir();
    // MOTORS
    motors_init();

    //constantly move forward.
	const int motorForward = 1000;
    left_motor_set_speed(motorForward);
    right_motor_set_speed(motorForward);

    /* Infinite loop. continuously avoid obstacles unless stuck, then turn.*/
    while (1) {

    	switch (mode) {
    	    case ROTATE_MODE:
				rotate(500,300);
    	        break;
    	    case NAVIGATE_MODE:
    	    	avoid_collisions(5, 150, motorForward, motorForward);
				chThdSleepMilliseconds(100);
				check_stationary();
    	        break;
    	    default:
    	    	chThdSleepMilliseconds(100);
    	    	break;
    	        // code to be executed if expression doesn't match any constant
    	}
    }
}

#define STACK_CHK_GUARD 0xe2dee396
uintptr_t __stack_chk_guard = STACK_CHK_GUARD;

void __stack_chk_fail(void)
{
    chSysHalt("Stack smashing detected");
}
