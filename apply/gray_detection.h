#ifndef __GRAY_DETECTION_H
#define __GRAY_DETECTION_H
#include "Headfile.h"




void gpio_input_init(void);
void gpio_input_check_channel_7(void);
void gray_turn_control_200hz(float *output);
void vision_turn_control_50hz(float *output);


void gpio_input_check_channel_12(void);
void gpio_input_check_from_vision(void);
void gpio_input_check_channel_12_with_handle(void);


extern float gray_status[2],gray_status_backup[2][20];
extern float rho_status[2],rho_status_backup[2][20];

extern float turn_scale,turn_output;
extern float rho_output,rho_output_last;
extern uint32_t gray_status_worse,vision_status_worse;


extern controller seektrack_ctrl[3];
extern float startpoint_straightaway_cm;

#endif
