/*
 * 	estimator.c
 *
 *  	Created on: Oct 5th 2025
 *      Author: Harry Lawton
 *
 *      Function:
 *      -> use sensor data to estimate the volume level inside the machine
 *      -> use sensor data to estimate the real bowl temperature of the liquid/slush
 *		-> use sensor data to estimate the texture index
 *		-> use sensor data to estimate the condenser load
 *		-> use sensor data to estimate the condenser load
 *
 *      Included:
 *      -> update_volume_estimate(void)
 *      -> update_bowl_temp_estimate(void)
 *      -> update_texture_index_estimate(void)
 *      -> update_condenser_load_estimate(void)
 *      -> update_icing_condition_estimate(void)
 *      -> update_slush_torque_estimate(void)
 *      -> update_freezing_point_estimate(void)
 *
 *      output variables:
 *      -> estimated_volume (zero, low, medium, high)
 *      -> estimated_bowl_temp (ºC)
 *      -> estimated_texture_index (0 - 5)
 *      -> estimated_condenser_load (0 - 100)
 *      -> estimated_icing_condition (none, low, medium, high)
 *      -> estimated_slush_torque (low, medium, high)
 *      -> estimated_freezing_point (low, medium, high)
 */

void estimator_init(void)
{
    /* Initialise state to zero; rely on first update to populate */

}

void estimator_update(uint32_t now_ms)
{
	update_volume_estimate(void)
	update_bowl_temp_estimate(void)
	update_texture_index_estimate(void)
	update_condenser_load_estimate(void)
	update_icing_condition_estimate(void)
	update_slush_torque_estimate(void)
	update_freezing_point_estimate(void)
}

void update_volume_estimate(void) {

}
void update_bowl_temp_estimate(void) {

}
void update_texture_index_estimate(void) {

}
void update_condenser_load_estimate(void) {

}
void update_icing_condition_estimate(void) {

}
void update_slush_torque_estimate(void) {

}
void update_freezing_point_estimate(void) {

}
