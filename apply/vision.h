#ifndef __VISION_H
#define __VISION_H

#define SDK_Target_Length  53//45
#include "Headfile.h"


// -------------------------视觉任务类型------------------------------
#define  Number_recognition_inbegin_task  0xA1  // 用于起初识别数字的任务
#define  Tracking_task  0xA2    				// 循迹任务
#define  Number_recognition_intrack_task  0xA3  // 用于十字路口识别数字的任务





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
	uint8_t cross;
	uint8_t fps;
	uint8_t camera_id;
	uint8_t task;   // 任务类别

	uint8_t inbegin_recognition_finsh_flag;  // 起初数字识别成功标志位
	uint8_t intrack_todo_task;   // 小车在赛道上应该执行的任务，向前，左转，右转

	

	uint16_t trust_cnt;
	uint16_t trust_flag;
	uint8_t line_ctrl_enable;
	uint8_t target_ctrl_enable;
	vector3f sdk_target,sdk_target_offset;
	float sdk_angle;
	split_state x0;

}Target_Check;//目标检测

void SDK_DT_Send_Check(uint8_t mode);

void SDK_Data_Receive_Prepare_1(uint8_t data);
void SDK_Data_Receive_Prepare_2(uint8_t data);

extern Target_Check camera1,camera2;


#endif

