#include "headfile.h"
#include "subtask.h"


#define flight_subtask_delta 5//5ms



uint16_t flight_subtask_cnt[SUBTASK_NUM]={0};//飞行任务子线程计数器，可以用于控制每个航点子线程的执行
uint32_t flight_global_cnt[SUBTASK_NUM]={0}; //飞行任务子线全局计数器，可以结合位置偏差用于判断判断航点是否到达
uint32_t execute_time_ms[SUBTASK_NUM]={0};//飞行任务子线执行时间，可以用于设置某个子线程的执行时间
uint32_t subtask_finish_flag[SUBTASK_NUM] = {0};  // 子任务完成flag





void subtask_reset(void)
{
	for(uint16_t i=0;i<SUBTASK_NUM;i++)
	{
		flight_subtask_cnt[i]=0;
		execute_time_ms[i]=0;
		flight_global_cnt[i]=0;
		subtask_finish_flag[i] = 0;
	}
}

/**
 * @brief 顺时针旋转90度任务
*/
void clockwise_rotate_90_task(void)		//顺时针转90度
{
	static uint8_t n = Clockwise_Rotation_90;	

	if(flight_subtask_cnt[n]==0)  // 子任务1：调正参数
	{
		trackless_output.yaw_ctrl_mode=CLOCKWISE;   // 偏航控制模式为相对偏航角度顺时针控制模式
		trackless_output.yaw_ctrl_start=1;			// 偏航控制开始标志位
		trackless_output.yaw_outer_control_output = 90;//顺时针90度	
		flight_subtask_cnt[n]++;   		// 执行下一阶段任务：

		speed_ctrl_mode = 1;
		steer_control(&turn_ctrl_pwm);  // 转向控制
		speed_setup = 0;
		//期望速度
		speed_expect[0]=speed_setup+turn_ctrl_pwm*steer_gyro_scale;//左边轮子速度期望
		speed_expect[1]=speed_setup-turn_ctrl_pwm*steer_gyro_scale;//右边轮子速度期望
		//速度控制
		speed_control_100hz(speed_ctrl_mode);	
	}
	else if(flight_subtask_cnt[n]==1)  // 等待工作模式变化
	{
		trackless_output.yaw_ctrl_mode=CLOCKWISE;
		trackless_output.yaw_outer_control_output  = 0;  // 偏航姿态控制器输入
		if(trackless_output.yaw_ctrl_end==1)
		{
			subtask_finish_flag[n] = 1;  // 任务已完成
		}

		speed_ctrl_mode = 1;
		steer_control(&turn_ctrl_pwm);  // 转向控制
		speed_setup = 0;
		//期望速度
		speed_expect[0]=speed_setup+turn_ctrl_pwm*steer_gyro_scale;//左边轮子速度期望
		speed_expect[1]=speed_setup-turn_ctrl_pwm*steer_gyro_scale;//右边轮子速度期望
		//速度控制
		speed_control_100hz(speed_ctrl_mode);	
	}

}

/**
 * @brief 逆时针旋转90度任务
*/
void contrarotate_90_task(void)		//逆时针转90°
{
	static uint8_t n = Contrarotate_90;	

	if(flight_subtask_cnt[n]==0)  // 子任务1：调正参数
	{
		trackless_output.yaw_ctrl_mode=ANTI_CLOCKWISE;   // 偏航控制模式为相对偏航角度顺时针控制模式
		trackless_output.yaw_ctrl_start=1;			// 偏航控制开始标志位
		trackless_output.yaw_outer_control_output = 90;//逆时针90度	
		flight_subtask_cnt[n]++;   		// 执行下一阶段任务：

		speed_ctrl_mode = 1;
		steer_control(&turn_ctrl_pwm);  // 转向控制
		speed_setup = 0;
		//期望速度
		speed_expect[0]=speed_setup+turn_ctrl_pwm*steer_gyro_scale;//左边轮子速度期望
		speed_expect[1]=speed_setup-turn_ctrl_pwm*steer_gyro_scale;//右边轮子速度期望
		//速度控制
		speed_control_100hz(speed_ctrl_mode);	
	}
	else if(flight_subtask_cnt[n]==1)  // 等待工作模式变化
	{
		trackless_output.yaw_ctrl_mode=ANTI_CLOCKWISE;
		trackless_output.yaw_outer_control_output  = 0;  // 偏航姿态控制器输入
		if(trackless_output.yaw_ctrl_end==1)
		{
			subtask_finish_flag[n] = 1;  // 任务已完成
		}

		speed_ctrl_mode = 1;
		steer_control(&turn_ctrl_pwm);  // 转向控制
		speed_setup = 0;
		//期望速度
		speed_expect[0]=speed_setup+turn_ctrl_pwm*steer_gyro_scale;//左边轮子速度期望
		speed_expect[1]=speed_setup-turn_ctrl_pwm*steer_gyro_scale;//右边轮子速度期望
		//速度控制
		speed_control_100hz(speed_ctrl_mode);	
	}

}


/**
 * @brief 速度控制
 * @param speed 速度大小，单位cm/s，可正可负
*/
void speed_control_task(int8_t speed)
{
	static uint8_t n = Speed_Control; 
	if(flight_subtask_cnt[n] == 0)
	{
		speed_ctrl_mode = 1;
		speed_expect[0]=speed_setup;//左边轮子速度期望
		speed_expect[1]=speed_setup;//右边轮子速度期望
	}
	else
	{
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
	static uint8_t n = Distance_Control; 
	
	if(flight_subtask_cnt[n] == 0)
	{
		distance_ctrl.expect=smartcar_imu.state_estimation.distance + distance;
		flight_subtask_cnt[n]++;
	}
	else
	{
		distance_control();
		speed_setup=distance_ctrl.output;
		speed_expect[0]=speed_setup;//左边轮子速度期望
		speed_expect[1]=speed_setup;//右边轮子速度期望
		speed_control_100hz(speed_ctrl_mode);
	}
}






// 任务3，送药任务
void deliver_medicine_task(void)
{
	static uint8_t n = 2;
	if(flight_subtask_cnt[n] == 0)	
	{
		distance_ctrl.expect=smartcar_imu.state_estimation.distance + 10.0f;
		flight_subtask_cnt[n]++;
	}

	else if(flight_subtask_cnt[n] == 1)
	{
		distance_control();

		speed_setup=distance_ctrl.output;

		speed_ctrl_mode=1;//速度控制方式为两轮单独控制
		speed_expect[0] = speed_setup;//左边轮子速度期望
		speed_expect[1] = speed_setup;//右边轮子速度期望
		//速度控制
		speed_control_100hz(speed_ctrl_mode);
		

		// 判断距离
		if(flight_global_cnt[n]<10)//连续N次满足位置偏差很小,即认为位置控制完成
		{
			if(ABS(distance_ctrl.error) < 1.0f)	flight_global_cnt[n]++;	
			else flight_global_cnt[n]/=2;		
		}
		else 
		{
			flight_global_cnt[n]=0;
			flight_subtask_cnt[n]++;  // 下一阶段任务：
		}
	}

	else if(flight_subtask_cnt[n]==2)  // 
		{
			trackless_output.yaw_ctrl_mode=ANTI_CLOCKWISE;   // 偏航控制模式为相对偏航角度顺时针控制模式
			trackless_output.yaw_ctrl_start=1;			// 偏航控制开始标志位
			trackless_output.yaw_outer_control_output = 90;//顺时针90度	
			flight_subtask_cnt[n]++;   		// 执行下一阶段任务：

			speed_ctrl_mode = 1;
			steer_control(&turn_ctrl_pwm);  // 转向控制
			speed_setup = 0;
			//期望速度
			speed_expect[0]=speed_setup+turn_ctrl_pwm*steer_gyro_scale;//左边轮子速度期望
			speed_expect[1]=speed_setup-turn_ctrl_pwm*steer_gyro_scale;//右边轮子速度期望
			//速度控制
			speed_control_100hz(speed_ctrl_mode);	

		}
		else if(flight_subtask_cnt[n]==3)  // 等待工作模式变化
		{
			trackless_output.yaw_ctrl_mode=ANTI_CLOCKWISE;
			trackless_output.yaw_outer_control_output  = 0;  // 偏航姿态控制器输入
			if(trackless_output.yaw_ctrl_end==1)
			{
				subtask_finish_flag[n] = 1;  // 任务已完成
			}


			speed_ctrl_mode = 1;
			steer_control(&turn_ctrl_pwm);  // 转向控制
			speed_setup = 0;
			//期望速度
			speed_expect[0]=speed_setup+turn_ctrl_pwm*steer_gyro_scale;//左边轮子速度期望
			speed_expect[1]=speed_setup-turn_ctrl_pwm*steer_gyro_scale;//右边轮子速度期望
			//速度控制
			speed_control_100hz(speed_ctrl_mode);	
			
		}
	else
	{
		speed_control_task(0);
	}
	// clockwise_rotation_90_subtask(n, flight_subtask_cnt[n]);
	
	
}
