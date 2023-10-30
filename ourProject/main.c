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
#include "sensors/proximity.h"
#include "epuck1x/uart/e_uart_char.h"
#include "serial_comm.h"
#include "motors.h"
#include "sensors/VL53L0X/VL53L0X.h"

messagebus_t bus;
MUTEX_DECL(bus_lock);
CONDVAR_DECL(bus_condvar);

int speed = 500;
unsigned int noise = 10;
int prox_sensor(int sensors_average[]) {
	/*
	 a method for returning the sensor number for the closest sensor to the object.
	 relying on the highest reading being the cloest value.
	 */
	for (int i = 0; i < 8; i++) {
		int count = 0;
		for (int j = 0; j < 8; j++) {
			if (sensors_average[i] > sensors_average[j]) {
				count++;
				if (count == 7)
					return i + 1;
			}
		}
	}
	return 0;
}

void distance(uint16_t val) {
	char str[100];
	int str_length;
	str_length = sprintf(str, "distance : %d", val);
	e_send_uart1_char(str, str_length);
}

void rotate(int speed, int time) {

	left_motor_set_speed(speed);
	right_motor_set_speed(-speed);

	chThdSleepMilliseconds(time);

	left_motor_set_speed(0);
	right_motor_set_speed(0);
}

void prox_reading(int num) {
	char str[100];
	int str_length;
	int val = get_calibrated_prox(num);
	str_length = sprintf(str, "Sensor Reading of %d is %d\n:", num, val);
	e_send_uart1_char(str, str_length);
}

void set_motor_speed(int left, int right) {
	left_motor_set_speed(left);
	right_motor_set_speed(right);
}

int main(void) {
	halInit();
	chSysInit();
	mpu_init();
	messagebus_init(&bus, &bus_lock, &bus_condvar);
	clear_leds();
	spi_comm_start();
	serial_start();
	proximity_start(SLOW_UPDATE);
	calibrate_ir();
	motors_init();
	VL53L0X_start();

	while (1) {

		// put a wait function here
		chThdSleepMilliseconds(100);

		int sensors[8];
		//get sensor readings.
		for (int i = 0; i < 8; i++) {
			sensors[i] = get_calibrated_prox(i);

			//remove noise.
			if (sensors[i] < noise) {
				sensors[i] = 0;
			}
		}

		//determine nearest sensor.
		int nearest = prox_sensor(sensors);

		if (nearest >= 1 && nearest <= 4) {
			set_motor_speed(speed, -speed);
			set_front_led(0);
			set_body_led(1);
		} else if (nearest >= 5 && nearest <= 6) {
			set_motor_speed(-speed, speed);
			set_front_led(0);
			set_body_led(1);
		} else if (nearest == 0 || nearest == 7) {
			uint16_t dist = VL53L0X_get_dist_mm();
			set_front_led(1);
			set_body_led(0);
			if (dist > 60 && dist < 100) {
				// define speed as non constant so that robot can avoid oscillations.
				set_motor_speed(12.5 * dist - 725, 12.5 * dist - 725);
				distance(dist);
			} else if (dist > 100) {
				set_motor_speed(+speed, +speed);
				distance(dist);
			} else if (dist < 30) {
				set_motor_speed(-speed, -speed);
				distance(dist);
			}
		} else {
			// find object from long distance code here!
			unsigned int intervalMax = 20;
			unsigned int intervalCount = 0;
			int rotationTime = 200;
			int rotationSpeed = 200;
			int cloestObject = 2000;
			unsigned int intervalsToClosestObject;

			while (intervalCount < intervalMax) {
				rotate(roationSpeed, rotationTime);
				uint16_t distance = VL53L0X_get_dist_mm();
				if (distance < cloestObject) {
					cloestObject = distance;
					intervalsToClosestObject = intervalCount;
				}
				intervalCount++;
			}

			rotate(-rotationSpeed, rotationTime * intervalsToClosestObject);

			set_motor_speed(speed, speed);
			chThdSleepMilliseconds(1000);
		}
	}
}

#define STACK_CHK_GUARD 0xe2dee396
uintptr_t __stack_chk_guard = STACK_CHK_GUARD;

void __stack_chk_fail(void) {
	chSysHalt("Stack smashing detected");
}
