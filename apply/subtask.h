#ifndef __SUBTASK_H
#define __SUBTASK_H


#define SUBTASK_NUM 20

// ----------------------任务类别------------------------
typedef enum
{
	Clockwise_Rotation_90 = 0,
	Contrarotate_90,
    Speed_Control,
    Distance_Control,
}Task_Type;



void subtask_reset(void);	



void speed_control_task(int8_t speed);
void distance_control_task(float distance);
void clockwise_rotate_90_task(void);		//顺时针转90度
void contrarotate_90_task(void);		//逆时针转90°




void deliver_medicine_task(void);   // 送药小车



extern uint32_t subtask_finish_flag[SUBTASK_NUM];



#endif
