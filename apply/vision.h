#ifndef __VISION_H
#define __VISION_H

#define SDK_Target_Length  53//45
#include "Headfile.h"

typedef enum 
{
	UART7_SDK=0,
	UART1_SDK,
}COM_SDK;


typedef union
{
	gray_flags gray;
	uint16_t value;
}split_state;

typedef struct
{
	uint16_t x;
	uint8_t flag;	
	uint8_t fps;
	uint8_t camera_id;
	uint8_t sdk_mode;


	float x_cm;
	float y_cm;
	float z_cm;	
	float x_pixel_size;
	float y_pixel_size;
	float z_pixel_size;
	float apriltag_distance;
	uint16_t trust_cnt;
	uint16_t trust_flag;
	uint8_t line_ctrl_enable;
	uint8_t target_ctrl_enable;
	vector3f sdk_target,sdk_target_offset;
	float sdk_angle;
	split_state x0;

}Target_Check;//目标检测

void SDK_Data_Receive_Prepare_1(uint8_t data);
void SDK_Data_Receive_Prepare_2(uint8_t data);

extern Target_Check camera1,camera2;


#endif

