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

#include "sensors/proximity.h"

#define PROX_SENSORS 8
static int distance_sensors[PROX_SENSORS];
static const char *distance_sensors_names[PROX_SENSORS] = {"ps0", "ps1", "ps2", "ps3", "ps4", "ps5", "ps6", "ps7"};

#define SENSOR_THRESHOLD 140

/* E-puck angular speed in rad/s */
#define MAX_SPEED 6.28

#define ROBOT_ANGULAR_SPEED_IN_DEGREES 283.588111888

static int TIME_STEP;

messagebus_t bus;
MUTEX_DECL(bus_lock);
CONDVAR_DECL(bus_condvar);

int left_motor;
int right_motor;

void robot_controller_init(int time_step)
{
	TIME_STEP = time_step;
  /* get a handler to the motors and set target position to infinity (speed control) */
  left_motor = wb_robot_get_device("left wheel motor");
  right_motor = wb_robot_get_device("right wheel motor");
  wb_motor_set_position(left_motor, INFINITY);
  wb_motor_set_position(right_motor, INFINITY);
  wb_motor_set_velocity(left_motor, 0.0);
  wb_motor_set_velocity(right_motor, 0.0);

  /* get a handler to the sensors */
	for (int i = 0; i < PROX_SENSORS; i++) {
			distance_sensors[i] = wb_robot_get_device(distance_sensors_names[i]);
			wb_distance_sensor_enable(distance_sensors[i], TIME_STEP);
	}
}

static float calculate_rotation_time(float degrees)
{
	return abs(degrees) / ROBOT_ANGULAR_SPEED_IN_DEGREES;
}

/* function to stop the motor (set motor velocity to zero) */
void motor_stop() {
  wb_motor_set_velocity(left_motor, 0);
  wb_motor_set_velocity(right_motor, 0);
}

/* function to set motor velocity to move forward */
void motor_move_forward() {
  wb_motor_set_velocity(left_motor, MAX_SPEED);
  wb_motor_set_velocity(right_motor, MAX_SPEED);
}

/* function to set motor velocity to rotate right in place*/
void motor_rotate_right() {
  wb_motor_set_velocity(left_motor, MAX_SPEED);
  wb_motor_set_velocity(right_motor, -MAX_SPEED);
}

/* function to set motor velocity to rotate left in place*/
void motor_rotate_left() {
  wb_motor_set_velocity(left_motor, -MAX_SPEED);
  wb_motor_set_velocity(right_motor, MAX_SPEED);
}

void motor_rotate_left_in_degrees(float degrees) {
	motor_rotate_left();

	float duration = calculate_rotation_time(degrees);
	float start_time = wb_robot_get_time();
	do
	{
		wb_robot_step(TIME_STEP);
	} while (wb_robot_get_time() < start_time + duration);

	motor_stop();
}

/* function to get sensors condition
 * if sensor detect obstacle, then the condition is true
 * */
bool * get_sensors_condition()
{
	static bool sensors_condition[PROX_SENSORS] = {false};

	for (int i = 0; i <  PROX_SENSORS; i++) {
		/*
		 * Obstacle detected condition is true if the sensor values is larger then the threshold value
		 * */
		if (wb_distance_sensor_get_value(distance_sensors[i]) > SENSOR_THRESHOLD) {
			sensors_condition[i] = true;
		} else {
			sensors_condition[i] = false;
		}
	}

	return sensors_condition;
}

/* function to print sensors values
 * */
void print_sensor_values() {
	printf("%s sensor values: ", wb_robot_get_name());

	for (int i = 0; i <  ; i++) {
		printf("%d:%.3f ", i, wb_distance_sensor_get_value(distance_sensors[i]));
	}

	printf("\n");
}

bool *get_sensors_condition()
{
	static bool sensors_condition[PROX_SENSORS] = {false};

	for (int i = 0; i < PROX_SENSORS ; i++) {
		/*
		 * Obstacle detected condition is true if the sensor values is larger then the threshold value
		 * */
		if (wb_distance_sensor_get_value(distance_sensors[i]) > SENSOR_THRESHOLD) {
			sensors_condition[i] = true;
		} else {
			sensors_condition[i] = false;
		}
	}

	return sensors_condition;
}

/* function to print sensors values
 * */
void print_sensor_values() {
	printf("%s sensor values: ", wb_robot_get_name());

	for (int i = 0; i <  ; i++) {
		printf("%d:%.3f ", i, wb_distance_sensor_get_value(distance_sensors[i]));
	}

	printf("\n");
}


int main(void)
{
	messagebus_init(&bus, &bus_lock, &bus_condvar);
	proximity_start();
	calibrate_ir();

    halInit();
    chSysInit();
    mpu_init();


//    MOTORS
    motors_init();



int i;
int motorForward = 500;
//int motorBack = -500;
//int motorStop = 0;

    /* Infinite loop. */
    while (1) {
    	bool *is_sensors_active = get_sensors_condition();

    			if (is_sensors_active[1] && is_sensors_active[6]) {
    				motor_rotate_left_in_degrees(180);
    			} else if (is_sensors_active[0] || is_sensors_active[1]) {
    				motor_rotate_left();
    			} else if (is_sensors_active[7] || is_sensors_active[6]) {
    				motor_rotate_right();
    			} else {
    				motor_move_forward();
    			}

//
//    	left_motor_set_speed(motorForward);
//    	right_motor_set_speed(motorBack);
//
//        chThdSleepMilliseconds(4000);
//
//    	left_motor_set_speed(motorStop);
//    	right_motor_set_speed(motorStop);
    }
}

#define STACK_CHK_GUARD 0xe2dee396
uintptr_t __stack_chk_guard = STACK_CHK_GUARD;

void __stack_chk_fail(void)
{
    chSysHalt("Stack smashing detected");
}
