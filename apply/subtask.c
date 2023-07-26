#include "headfile.h"
#include "subtask.h"

#define SUBTASK_NUM 20
#define flight_subtask_delta 5//5ms



uint16_t flight_subtask_cnt[SUBTASK_NUM]={0};//飞行任务子线程计数器，可以用于控制每个航点子线程的执行
uint32_t flight_global_cnt[SUBTASK_NUM]={0}; //飞行任务子线全局计数器，可以结合位置偏差用于判断判断航点是否到达
uint32_t execute_time_ms[SUBTASK_NUM]={0};//飞行任务子线执行时间，可以用于设置某个子线程的执行时间

void subtask_reset(void)
{
	for(uint16_t i=0;i<SUBTASK_NUM;i++)
	{
		flight_subtask_cnt[i]=0;
		execute_time_ms[i]=0;
		flight_global_cnt[i]=0;
	}
}

// 任务0
void clockwise_rotation_90(void)//顺时针转90°
{
	static uint8_t n=0;		// 任务0
	if(flight_subtask_cnt[n]==0)
	{
		trackless_output.yaw_ctrl_mode=CLOCKWISE;   // 偏航控制模式为相对偏航角度顺时针控制模式
		trackless_output.yaw_ctrl_start=1;			// 偏航控制开始标志位
		trackless_output.yaw_outer_control_output = 90;//顺时针90度	
		flight_subtask_cnt[n]=1;   		// 执行下一阶段任务：
	}
	else if(flight_subtask_cnt[n]==1)
	{
		trackless_output.yaw_ctrl_mode=CLOCKWISE;
		trackless_output.yaw_outer_control_output  = 0;  // 偏航姿态控制器输入
		
		// if(trackless_output.yaw_ctrl_end==1)  flight_subtask_cnt[n]=2;//执行完毕后，切换到下一阶段
	}
	// else if(flight_subtask_cnt[n]==2)
	// {
	// 	trackless_output.yaw_ctrl_mode=ROTATE;   // 手动偏航控制模式
	// 	trackless_output.yaw_outer_control_output  = RC_Data.rc_rpyt[RC_ROLL];
	// }
	// else//其它情形
	// {
	// 	trackless_output.yaw_ctrl_mode=ROTATE;
	// 	trackless_output.yaw_outer_control_output  = RC_Data.rc_rpyt[RC_ROLL];		
	// }
}


// 任务1
void contrarotate_90(void)//逆时针转90°
{
	static uint8_t n=1;
	if(flight_subtask_cnt[n]==0)
	{
		trackless_output.yaw_ctrl_mode=ANTI_CLOCKWISE;
		trackless_output.yaw_ctrl_start=1;
		trackless_output.yaw_outer_control_output  = 90;//逆时针90度,传递期望
		flight_subtask_cnt[n]=1;		
	}
	else if(flight_subtask_cnt[n]==1)
	{
		trackless_output.yaw_ctrl_mode=ANTI_CLOCKWISE;
		trackless_output.yaw_outer_control_output  = 0;
		
		if(trackless_output.yaw_ctrl_end==1)  flight_subtask_cnt[n]=2;//执行完毕后，切换到下一阶段	
	}
	// else if(flight_subtask_cnt[n]==2)
	// {
	// 	trackless_output.yaw_ctrl_mode=ROTATE;
	// 	trackless_output.yaw_outer_control_output  =RC_Data.rc_rpyt[RC_ROLL];
	// }
	else//其它情形
	{
		trackless_output.yaw_ctrl_mode=ROTATE;
		trackless_output.yaw_outer_control_output = smartcar_imu.rpy_deg[_YAW];		
	}
}


void clockwise_rotation_90_task(uint8_t speed)//顺时针转90°任务
{
	speed_ctrl_mode=1;//速度控制方式为两轮单独控制			
	clockwise_rotation_90();
	steer_control(&turn_ctrl_pwm);
	speed_setup = speed;
	//期望速度
	speed_expect[0]=speed_setup+turn_ctrl_pwm*steer_gyro_scale;//左边轮子速度期望
	speed_expect[1]=speed_setup-turn_ctrl_pwm*steer_gyro_scale;//右边轮子速度期望
	//速度控制
	speed_control_100hz(speed_ctrl_mode);		


}

void contrarotate_90_task(uint8_t speed)//逆时针转90°任务
{
	speed_ctrl_mode=1;//速度控制方式为两轮单独控制			
	contrarotate_90();
	steer_control(&turn_ctrl_pwm);
	speed_setup = speed;	
	//期望速度
	speed_expect[0]=speed_setup+turn_ctrl_pwm*steer_gyro_scale;//左边轮子速度期望
	speed_expect[1]=speed_setup-turn_ctrl_pwm*steer_gyro_scale;//右边轮子速度期望
	//速度控制
	speed_control_100hz(speed_ctrl_mode);	

}

/**
 * @brief 速度控制
 * @param speed 速度大小，单位cm/s，可正可负
*/
void speed_control_task(int8_t speed)
{
	if(speed >= 0)
	{
		trackless_motor.right_motion_dir_config = 0;
		trackless_motor.left_motion_dir_config = 0;
		speed_ctrl_mode=1;//速度控制方式为两轮单独控制
		speed_expect[0]=speed;//左边轮子速度期望
		speed_expect[1]=speed;//右边轮子速度期望
		//速度控制
		speed_control_100hz(speed_ctrl_mode);
	}
	else
	{
		trackless_motor.right_motion_dir_config = 1;
		trackless_motor.left_motion_dir_config = 1;
		speed_ctrl_mode=1;//速度控制方式为两轮单独控制
		speed_expect[0]=ABS(speed);//左边轮子速度期望
		speed_expect[1]=ABS(speed);//右边轮子速度期望
		//速度控制
		speed_control_100hz(speed_ctrl_mode);
	}
		

}
/**
 * @brief 距离控制
 * @param distance 前进或者后退的距离
*/
void distance_control_task(float distance)
{
	distance_ctrl.expect=smartcar_imu.state_estimation.distance + distance;
	distance_control();
	speed_setup=distance_ctrl.output;
	//期望速度
	speed_expect[0]=speed_setup;//左边轮子速度期望
	speed_expect[1]=speed_setup;//右边轮子速度期望
	speed_control_100hz(speed_ctrl_mode);
}



// 任务3，送药任务
void deliver_medicine_task(void)
{
	static uint8_t n = 2;
	if(flight_subtask_cnt[n] == 0)	// 初始任务，查看数字
	{
		// 速度0控制
		speed_ctrl_mode=1;//速度控制方式为两轮单独控制
		speed_expect[0]=0;//左边轮子速度期望
		speed_expect[1]=0;//右边轮子速度期望
		//速度控制
		speed_control_100hz(speed_ctrl_mode);
		
		// 查看数字任务 to_do
		
		flight_subtask_cnt[n]++;		// 下一阶段任务：物体检测
	}

	else if(flight_subtask_cnt[n] == 1)	// 物体检测
	{
		flight_subtask_cnt[n]++;	// 下一阶段任务：循迹任务
	}

	else if(flight_subtask_cnt[n] == 2)  	// 循迹任务
	{
		
	}

}
