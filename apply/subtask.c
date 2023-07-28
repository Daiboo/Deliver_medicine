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


// ----------------------------------------送药小车任务------------------------------------------



Deliver_medicine_task_param __deliver_medicine_task_param = {
	.speed = deliver_medicine_car_speed_default,
	.fix_rotate_point = fix_rotate_point_default,
	
};


typedef enum
{
	inbegin_number_recognition_task_state = 0,  // 状态：起初数字识别任务
	tracking_control_until_recognition_cross_or_stop = 1,  // 状态：循迹控制，直到识别到十字或停止位
	speed0_control_until_receive_todo = 2,    // 状态：零速度控制，直到收到前左右转指令

	speed0_control = 3,   		// 0速度控制

	clockwise_rotate_90_task_state = 100,  // 左转状态机
	contrarotate_90_task_state = 120,		// 右转状态机
	
}Task_state;  // 状态机阶段

// -----------------------------------------送药任务状态机---------------------------------
void deliver_medicine_task(void)
{
	static uint8_t n = Deliver_Medicine;
	// static uint8_t Car_Intrack_todo_task = 0;

// ------------------------------状态：起初数字识别任务-------------------------------
	if(flight_subtask_cnt[n] == inbegin_number_recognition_task_state)// 状态：起初数字识别任务，直到收到完成标志位才转移
	{
		SDK_DT_Send_Check(Number_recognition_inbegin_task, (COM_SDK)7);  // 发送起初数字识别任务给openmv
		if(camera1.inbegin_recognition_finsh_flag)
		{
			flight_subtask_cnt[n] = tracking_control_until_recognition_cross_or_stop;
			SDK_DT_Send_Check(Tracking_task, (COM_SDK)7);  // 发送循迹任务给openmv

			beep.period = 200;
			beep.light_on_percent = 0.5f;
			beep.reset = 1;
			beep.times = 2;
		}
	}

// ------------------------------- 状态：循迹控制，直到识别到十字或停止位--------------------------------------
	else if(flight_subtask_cnt[n] == tracking_control_until_recognition_cross_or_stop) // 状态：循迹控制 直到识别到十字
	{
		speed_ctrl_mode=1;  //速度控制方式为两轮单独控制
		vision_turn_control_50hz(&turn_ctrl_pwm);
		speed_setup = __deliver_medicine_task_param.speed;
		speed_expect[0]=speed_setup+turn_ctrl_pwm*turn_scale;//左边轮子速度期望
		speed_expect[1]=speed_setup-turn_ctrl_pwm*turn_scale;//右边轮子速度期望
		//速度控制
		speed_control_100hz(speed_ctrl_mode);
		if(camera1.cross == 1)   // 如果检测到十字
		{
			SDK_DT_Send_Check(Number_recognition_intrack_task,(COM_SDK)7);		// 发送赛道数字识别任务给openmv

			beep.period = 200;
			beep.light_on_percent = 0.5f;
			beep.reset = 1;
			beep.times = 1;

			flight_subtask_cnt[n] = speed0_control_until_receive_todo;
		}
		else		// 如果检测到停止位
		{
			flight_subtask_cnt[n] = speed0_control;
		}
	}
// -----------------------状态：零速度控制，直到收到前左右转指令-------------------
	else if(flight_subtask_cnt[n] == speed0_control_until_receive_todo) // 状态：零速度控制，直到收到前进，左转，右转指令
	{
		// 零速度控制
		speed_ctrl_mode=1; 
		speed_expect[0] = 0;	//左边轮子速度期望
		speed_expect[1] = 0;	//右边轮子速度期望
		speed_control_100hz(speed_ctrl_mode);

		if(camera1.intrack_todo_task == 4)   // 100 左转 
		{	
			beep.period = 200;
			beep.light_on_percent = 0.5f;
			beep.reset = 1;
			beep.times = 3;

			flight_subtask_cnt[n] = contrarotate_90_task_state;  // 下一状态：左转90度
		}
		else if(camera1.intrack_todo_task == 0)  // 000 直走  
		{
			flight_subtask_cnt[n] = tracking_control_until_recognition_cross_or_stop; // 下一状态；循迹控制直到识别十字
			
			beep.period = 200;
			beep.light_on_percent = 0.5f;
			beep.reset = 1;
			beep.times = 3;
		}
		else if(camera1.intrack_todo_task == 1)  // 001 右转
		{
			flight_subtask_cnt[n] = clockwise_rotate_90_task_state;  // 下一状态：右转90度

			beep.period = 200;
			beep.light_on_percent = 0.5f;
			beep.reset = 1;
			beep.times = 3;
		}
	}



// ---------------------------------左转状态机------------------------------------
	else if(flight_subtask_cnt[n] == contrarotate_90_task_state)
	{
		distance_ctrl.expect=smartcar_imu.state_estimation.distance 
		+ __deliver_medicine_task_param.fix_rotate_point;  		// 修正转向基准点

		flight_subtask_cnt[n]++;
	}
	else if(flight_subtask_cnt[n] == contrarotate_90_task_state + 1)
	{
		distance_control();
		speed_setup=distance_ctrl.output;

		speed_ctrl_mode=1;//速度控制方式为两轮单独控制
		speed_expect[0] = speed_setup;//左边轮子速度期望
		speed_expect[1] = speed_setup;//右边轮子速度期望
		//速度控制
		speed_control_100hz(speed_ctrl_mode);

		// 判断距离
		if(flight_global_cnt[n] < target_point_fit_times)//连续N次满足位置偏差很小,即认为位置控制完成
		{
			if(ABS(distance_ctrl.error) < distance_precision_cm)flight_global_cnt[n]++;	
			else flight_global_cnt[n]/=2;		
		}
		else 
		{
			flight_global_cnt[n]=0;
			flight_subtask_cnt[n]++;  // 下一阶段任务：
		}

	}
	else if(flight_subtask_cnt[n] == contrarotate_90_task_state + 2)  // 状态：转向期望设置
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

	else if(flight_subtask_cnt[n] == contrarotate_90_task_state + 3)  // 状态：转向控制，直到满足
	{
			trackless_output.yaw_ctrl_mode=ANTI_CLOCKWISE;
			trackless_output.yaw_outer_control_output  = 0;  // 偏航姿态控制器输入
			speed_ctrl_mode = 1;
			steer_control(&turn_ctrl_pwm);  // 转向控制
			speed_setup = 0;
			//期望速度
			speed_expect[0]=speed_setup+turn_ctrl_pwm*steer_gyro_scale;//左边轮子速度期望
			speed_expect[1]=speed_setup-turn_ctrl_pwm*steer_gyro_scale;//右边轮子速度期望
			//速度控制
			speed_control_100hz(speed_ctrl_mode);

			if(trackless_output.yaw_ctrl_end==1)
			{
				flight_subtask_cnt[n] = tracking_control_until_recognition_cross_or_stop; 	// 下一状态：
			}
	}


// ------------------------------------右转状态机---------------------------------------
	else if(flight_subtask_cnt[n] == clockwise_rotate_90_task_state)
	{
		distance_ctrl.expect=smartcar_imu.state_estimation.distance 
		+ __deliver_medicine_task_param.fix_rotate_point;  		// 修正转向基准点

		flight_subtask_cnt[n]++;
	}
	else if(flight_subtask_cnt[n] == clockwise_rotate_90_task_state + 1)
	{
		distance_control();
		speed_setup=distance_ctrl.output;

		speed_ctrl_mode=1;//速度控制方式为两轮单独控制
		speed_expect[0] = speed_setup;//左边轮子速度期望
		speed_expect[1] = speed_setup;//右边轮子速度期望
		//速度控制
		speed_control_100hz(speed_ctrl_mode);

		// 判断距离
		if(flight_global_cnt[n] < target_point_fit_times)//连续N次满足位置偏差很小,即认为位置控制完成
		{
			if(ABS(distance_ctrl.error) < distance_precision_cm)flight_global_cnt[n]++;	
			else flight_global_cnt[n]/=2;		
		}
		else 
		{
			flight_global_cnt[n]=0;
			flight_subtask_cnt[n]++;  
		}

	}
	else if(flight_subtask_cnt[n] == clockwise_rotate_90_task_state + 2)  // 状态：转向期望设置
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

	else if(flight_subtask_cnt[n] == contrarotate_90_task_state + 3)  // 状态：转向控制，直到满足
	{
		trackless_output.yaw_ctrl_mode=CLOCKWISE;
		trackless_output.yaw_outer_control_output  = 0;  // 偏航姿态控制器输入
		speed_ctrl_mode = 1;
		steer_control(&turn_ctrl_pwm);  // 转向控制
		speed_setup = 0;
		//期望速度
		speed_expect[0]=speed_setup+turn_ctrl_pwm*steer_gyro_scale;//左边轮子速度期望
		speed_expect[1]=speed_setup-turn_ctrl_pwm*steer_gyro_scale;//右边轮子速度期望
		//速度控制
		speed_control_100hz(speed_ctrl_mode);

		if(trackless_output.yaw_ctrl_end==1)
		{
			flight_subtask_cnt[n] = tracking_control_until_recognition_cross_or_stop; 	// 下一状态：
		}
	}

// ------------------------------------0速度控制状态机----------------------------------
	else if(flight_subtask_cnt[n] == speed0_control)
	{
		// 零速度控制
		speed_ctrl_mode=1; 
		speed_expect[0] = 0;	//左边轮子速度期望
		speed_expect[1] = 0;	//右边轮子速度期望
		speed_control_100hz(speed_ctrl_mode);

	}





// -------------------------------------


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
