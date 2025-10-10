/*
 * 	estimator.c
 *
 *  	Created on: Oct 5th 2025
 *      Author: Harry Lawton
 *
 *      Function:
 *      -> output the number of hours the motor has run onto the RGB led in a coded pattern
 *      -> this section will be added to with additional functionality on servicing.
 *      -> save the motor hours to device memory
 *
 *      Included:
 *      -> get_motor_hours(void)
 *      -> run_motor_hours_pattern(void)
 *      -> save_motor_hours](void)
 *
 */


void get_motor_hours()
{
	//get motor hours from memory and round to the nearest 100 hours
}

void run_motor_hours_pattern(uint8_t motor_hours)
{
	//control RGB LED to project the pattern that corrispond to the input number of horus
	//convert number to pattern code and update LED until machine exits service state.
}

void save_motor_hours()
{
    // when called the number of hours the machine has recorded the motor running for is updated in memory.

}
