#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "ch.h"
#include "hal.h"
#include "memory_protection.h"
#include <main.h>
#include "leds.h"
#include "spi_comm.h"

int main(void)
{

    halInit();
    chSysInit();
    mpu_init();

    void clear_leds(void);
    void spi_comm_start(void);


    /* Infinite loop. */
    while (1) {
    	unsigned int ledValue = 2;  // or any other non-negative integer value
    	set_body_led(ledValue);
        chThdSleepMilliseconds(1000);

    }
}

#define STACK_CHK_GUARD 0xe2dee396
uintptr_t __stack_chk_guard = STACK_CHK_GUARD;

void __stack_chk_fail(void)
{
    chSysHalt("Stack smashing detected");
}
