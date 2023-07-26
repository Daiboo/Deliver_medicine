#ifndef __ULTRASONIC_H
#define __ULTRASONIC_H
#include <stdint.h>

typedef struct
{
	uint32_t rf_start_time;
	uint32_t rf_end_time;
	uint32_t rf_delta;
	uint8_t rf_update_flag;
	float distance;
	float last_distance;
	float pre_last_distance;
	float vel;
	float acc;
	float last_vel;
	uint32_t rf_rssi_cnt;
	uint16_t cnt;
	uint8_t sensor_type;
	uint8_t sensor_init_type;
}_rangefinder;

void hcsr04_init(void);
void rangefinder_init(void);

void us100_statemachine(void);
void hcsr04_statemachine(void);
void rangefinder_statemachine(void);


extern uint8_t  com5_rx_buf[4];
extern uint16_t com5_rx_cnt;
extern _rangefinder rangefinder;

#endif
