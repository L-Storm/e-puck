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

int speed = 400;

int prox_sensor(int sensors_average[]) {
    for (int i = 0; i < 8; i++) {
        int count = 0;
        for (int j = 0; j < 8; j++) {
            if (sensors_average[i] > sensors_average[j]) {
                count++;
                if (count == 7) return i + 1;
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
        int sensors[8];
        for (int i = 0; i < 8; i++) {
            sensors[i] = get_calibrated_prox(i);
        }

        int sensors_sum[8] = {0};
        int sensors_avg[8];

        for (int i = 0; i < 8; i++) {
            for (int j = 0; j < 10; j++) {
                sensors_sum[i] += sensors[i];
            }
            sensors_avg[i] = sensors_sum[i] / 10;
        }

        int nearest = prox_sensor(sensors_avg);

        if (nearest >= 2 && nearest <= 4) {
            set_motor_speed(speed, -speed);
            set_front_led(0);
            set_body_led(1);
        } else if (nearest >= 5 && nearest <= 7) {
            set_motor_speed(-speed, speed);
            set_front_led(0);
            set_body_led(1);
        } else {
            uint16_t dist = VL53L0X_get_dist_mm();
            set_front_led(1);
            set_body_led(0);
            if (dist > 30) {
                set_motor_speed(speed, speed);
                distance(dist);
            } else if (dist < 20) {
                set_motor_speed(-speed, -speed);
                distance(dist);
            }
        }
    }
}

#define STACK_CHK_GUARD 0xe2dee396
uintptr_t __stack_chk_guard = STACK_CHK_GUARD;

void __stack_chk_fail(void) {
    chSysHalt("Stack smashing detected");
}
