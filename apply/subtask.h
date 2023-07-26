#ifndef __SUBTASK_H
#define __SUBTASK_H





void subtask_reset(void);	



void clockwise_rotation_90_task(uint8_t speed); //顺时针转90°任务
void contrarotate_90_task(uint8_t speed);//逆时针转90°任务
void speed_control_task(int8_t speed);		// 速度控制任务
void distance_control_task(float distance);  // 距离控制任务

void deliver_medicine_task(void);   // 送药小车







#endif
