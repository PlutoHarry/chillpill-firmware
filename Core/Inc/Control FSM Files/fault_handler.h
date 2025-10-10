/*
 * temperature_pid.h
 *
 *  Created on: Apr 7, 2023
 *      Author: DELL
 */

#ifndef INC_PID_CONTROLLER_H_
#define INC_PID_CONTROLLER_H_
#include "stdbool.h"

typedef enum
{

	FREEZE_TEMP_PLUS4,
	FREEZE_TEMP_MINUS1,
	FREEZE_TEMP_MINUS3,
	FREEZE_TEMP_MINUS6,
	FREEZE_TEMP_NONE,
	FREEZE_TEMP_NUM,
}freeze_temp_set_t;

#define TEMP_PID_FAST  2
#define TEMP_PID_LOW   1
#define TEMP_PID_STOP  0

#define TEMP_SETPOINT_VALUE_NONE   100
#define TEMP_SETPOINT_VALUE_PLUS4    4
#define TEMP_SETPOINT_VALUE_MINUS3  -3
#define TEMP_SETPOINT_VALUE_MINUS1  -1
#define TEMP_SETPOINT_VALUE_MINUS6  -6

#define NTC_TEMP_ONE   0
#define NTC_TEMP_TWO   1
#define NTC_TEMP_THREE 2
#define NTC_TEMP_FOUR  3

#define COMPRESSOR_TOO_COLD_DIFF  6

void temperature_pid_set_setpoint(freeze_temp_set_t temp);
int32_t temperature_pid_get_setpoint(void);
void temperature_pid_time_to_run_set(bool flag);
bool temperature_pid_time_to_run_get(void);
void inverter_controller(uint8_t target_frequency);
uint8_t temperature_check_compressor(void);

#endif /* INC_PID_CONTROLLER_H_ */
