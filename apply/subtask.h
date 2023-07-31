#ifndef __SUBTASK_H
#define __SUBTASK_H


#define SUBTASK_NUM 20

// ----------------------任务类别------------------------
typedef enum
{
	Speed_Control = 0,
	Distance_Control,
	Contrarotate_90,
	Clockwise_Rotation_90,
    Deliver_Medicine,
	Car_Stop,
}Task_Type;

// ----------------------------------------送药小车任务------------------------------------------



#define deliver_medicine_car_speed_default 30.0f
#define start_point_precision_cm 0.5f
#define fix_rotate_point_default 10.0f
#define start_point_fit_times   20//连续n次满足位置偏差很小,即认为位置控制完成
#define distance_precision_cm 1.0f
#define target_point_fit_times   10
#define tracking_distance_forward_default 40.0f


typedef struct 
{
	float speed;			// 任务中小车的速度
	float fix_rotate_point;  // 任务中距离点修正
	float tracking_distance_forward;  // 任务中循迹同时距离控制前进的距离
}Deliver_medicine_task_param;


extern Deliver_medicine_task_param __deliver_medicine_task_param;



void subtask_reset(void);	
void subtask_thread_reset(Task_Type task);


void speed_control_task(int8_t speed);
void distance_control_task(float distance);
void clockwise_rotate_90_task(void);		//顺时针转90度
void contrarotate_90_task(void);		//逆时针转90°




void deliver_medicine_task(void);   // 送药小车



extern uint32_t subtask_finish_flag[SUBTASK_NUM];



#endif
