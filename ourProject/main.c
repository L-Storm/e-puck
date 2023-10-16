#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "ch.h"
#include "hal.h"
#include "memory_protection.h"
#include <main.h>

//LEDS
#include "leds.h"
#include "spi_comm.h"

//MOTORS
#include "motors.h"

int main(void)
{

    halInit();
    chSysInit();
    mpu_init();

//    LEDS
    clear_leds();
    spi_comm_start();

//    MOTORS
    motors_init();

int i;
int motorForward = 500;
int motorBack = -500;
int motorStop = 0;

    /* Infinite loop. */
    while (1) {



    	for(i=1; i<=4; i++){
        	unsigned int ledValue = 2;  // or any other non-negative integer value
        	set_body_led(ledValue);
            chThdSleepMilliseconds(1000);
    	}


    	left_motor_set_speed(motorForward);
    	right_motor_set_speed(motorBack);

        chThdSleepMilliseconds(4000);

    	left_motor_set_speed(motorStop);
    	right_motor_set_speed(motorStop);
    }
}

#define STACK_CHK_GUARD 0xe2dee396
uintptr_t __stack_chk_guard = STACK_CHK_GUARD;

void __stack_chk_fail(void)
{
    chSysHalt("Stack smashing detected");
}
