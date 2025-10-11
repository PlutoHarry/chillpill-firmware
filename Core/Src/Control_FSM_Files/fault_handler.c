/*
 * 	fault_handler.c
 *
 *  	Created on: Oct 5th 2025
 *      Author: Harry Lawton
 *
 *      Function:
 *      -> detect when conditions are met that indicate a fault in one of the systems
 *      -> respond to that fault appropriately and restore the machine out of fault conditions
 *		-> communicate the specific fault to the user
 *
 *      Included:
 *      -> run_fault_detection(void)
 *      -> detect_motor_fault(void)
 *      -> detect_fan_fault(void)
 *      -> detect_compressor_fault(void)
 *      -> detect_sensor_fault(void)
 *      -> motor_fault_handler(void)
 *      -> fan_fault_handler(void)
 *      -> compressor_fault_handler(void)
 *      -> set_rgb_fault_lights()
 *
 */

extern IWDG_HandleTypeDef hiwdg;

void run_fault_detection()
{
	detect_motor_fault(void);
	detect_fan_fault(void);
	detect_compressor_fault(void);
}

void detect_motor_fault()
{
//if motor speed has droped below 8RPM for more then a 0.5s
//if motor current has exceeded 2.2A for over 1 second
	//trigger motor fault
	if (motor_fault_state = true) {
		motor_fault_handler();
	}
}

void detect_compressor_fault()
{
//if compressor is supposed to be running but the temperature sensors are not detecting any cooling trigger fault
	//if temperature sensors do not see a more then 5ºC drop within 4 minuets at the inlet to the evaporator the comprressor. This only runs on first start up.
}

void detect_fan_fault()
{
//if the fan encoders are reading that the fans are not running when they are supposed to be on then the system should trigger a fault.

}

void motor_fault_handler()
{
//disable the motor
	//set rgb colour code
	//try to reset the motor and sensor data and try again
}

void compressor_fault_handler()
{
//stop the compressor running and motor. keep fans running at last state
	//show the rgb code
	//wait 4 mins and try to restart
	//if it works then return to previous working
	//try 3 times. if it does not work then turn off the machine.
}

void fan_fault_handler()
{
	//before stopping the rest of the system like in the other faults, try a quick stop and start reset. Reset the sensor as well.
		//if the fault persists for 20 seconds after the reset then the fault should be escalated.
	//if escalated then the compressor and motor should turn off. the front rgb should show the pattern and then
	//try to reset the fan and sensor data again waiting 30 seconds between trys.
	//Keep trying for 5 cycles. if it doesn't work to fix the problem then turn off the machine.
	//if it does work then return the machine to normal fucntion before the disruption.
}
