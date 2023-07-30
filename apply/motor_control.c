#include "Headfile.h"
#include "motor_control.h"
#include "user.h"



float left_pwm,right_pwm;//左右电机最终的输出值

motor_config trackless_motor = {
	.left_encoder_dir_config = left_motor_encoder_dir_default,
	.right_encoder_dir_config = right_motor_encoder_dir_default,				//编码器方向配置
	.left_motion_dir_config = left_motion_dir_default,
	.right_motion_dir_config = right_motion_dir_default, 		//电机运动方向配置
	.wheel_radius_cm = tire_radius_cm_default,				//轮胎半径,单位为cm
	.pulse_num_per_circle = pulse_cnt_per_circle_default,			//轮胎转动一圈累计的脉冲数量

};

uint8_t speed_ctrl_mode=1;//0:直接控制、1:两轮单独控制、2:自平衡控制模式
uint8_t speed_pid_mode = 0;  // 0:位置式pid控制， 1：增量式pid控制
float motion_ctrl_pwm = 0, turn_ctrl_pwm = 0;  // 运动转向控制输出
int16_t motion_test_pwm_default = 500;
 

#define speed_err_limit 100.0f      // 自平衡速度偏差限幅值
#define speed_integral_limit 500.f  // 自平衡积分限幅值
#define speed_ctrl_output_limit 750.0f  // 自平衡控制器限幅值

#define speed_err_max  50.0f				//速度偏差限幅值
#define speed_integral_max  600.0f	//积分限幅值
#define speed_ctrl_output_max 999		//控制器限幅值



// 速度控制器参数
float speed_setup = 0;   // 速度设定值
float speed_kp = speed_kp_default, speed_ki = speed_ki_default, speed_kd = speed_kd_default;
float speed_error[2] = {0, 0}, speed_expect[2] = {speed_expect_default, speed_expect_default},speed_feedback[2] = {0, 0};
float speed_integral[2] = {0, 0}, speed_output[2] = {0, 0};




#define speed_smooth_period   (20/5)//20ms
uint16_t speed_smooth_output_cnt=0;
float speed_smooth_output=0;


float speed_output_delta[2]={0,0},speed_output_last[2]={0,0},speed_error_last[2]={0,0};


/***************************************************
函数名: void speed_control_100hz(uint8_t _speed_ctrl_mode)
说明:	速度控制
入口:	uint8_t _speed_ctrl_mode-速度控制方式,1:普通速度控制;2:自平衡速度控制
出口:	无
备注:	无
作者:	无名创新
****************************************************/
void speed_control_100hz(uint8_t _speed_ctrl_mode)
{
	static uint16_t _cnt=0;
	switch(_speed_ctrl_mode)
	{
		case 1:  // 普通速度控制
		{
			_cnt++; if(_cnt < 2) return; _cnt = 0;  // 10ms控制一次
			if(speed_pid_mode == 0)   // 位置式pid控制
			{
				speed_feedback[0] = smartcar_imu.left_motor_speed_cmps;
				speed_error[0]=speed_expect[0]-speed_feedback[0];
				speed_error[0]=constrain_float(speed_error[0],-speed_err_max,speed_err_max);
				speed_integral[0]+=speed_ki*speed_error[0];
				speed_integral[0]=constrain_float(speed_integral[0],-speed_integral_max,speed_integral_max);
				speed_output[0]=speed_integral[0]+speed_kp*speed_error[0];
				speed_output[0]=constrain_float(speed_output[0],-speed_ctrl_output_max,speed_ctrl_output_max);
				
				speed_feedback[1]=smartcar_imu.right_motor_speed_cmps;
				speed_error[1]=speed_expect[1]-speed_feedback[1];
				speed_error[1]=constrain_float(speed_error[1],-speed_err_max,speed_err_max);
				speed_integral[1]+=speed_ki*speed_error[1];
				speed_integral[1]=constrain_float(speed_integral[1],-speed_integral_max,speed_integral_max);
				speed_output[1]=speed_integral[1]+speed_kp*speed_error[1];
				speed_output[1]=constrain_float(speed_output[1],-speed_ctrl_output_max,speed_ctrl_output_max);

			}
			else   // 增量式pid
			{
				speed_output_last[0]=speed_output[0];//记录上次控制器输出
				speed_error_last[0]=speed_error[0];  //记录上次偏差
				speed_feedback[0]=smartcar_imu.left_motor_speed_cmps;
				speed_error[0]=speed_expect[0]-speed_feedback[0];
				speed_error[0]=constrain_float(speed_error[0],-speed_err_max,speed_err_max);
				speed_output_delta[0]=speed_kp*(speed_error[0]-speed_error_last[0])+speed_ki*speed_error[0];//计算速度输出增量
				speed_output[0]=speed_output_last[0]+speed_output_delta[0];//当前控制器输出
				speed_output[0]=constrain_float(speed_output[0],-speed_ctrl_output_max,speed_ctrl_output_max);
				
				
				
				speed_output_last[1]=speed_output[1];//记录上次控制器输出
				speed_error_last[1]=speed_error[1];  //记录上次偏差
				speed_feedback[1]=smartcar_imu.right_motor_speed_cmps;
				speed_error[1]=speed_expect[1]-speed_feedback[1];
				speed_error[1]=constrain_float(speed_error[1],-speed_err_max,speed_err_max);
				speed_output_delta[1]=speed_kp*(speed_error[1]-speed_error_last[1])+speed_ki*speed_error[1];//计算速度输出增量
				speed_output[1]=speed_output_last[1]+speed_output_delta[1];//当前控制器输出
				speed_output[1]=constrain_float(speed_output[1],-speed_ctrl_output_max,speed_ctrl_output_max);			
			
			}
			
		}
		break;

			
	}

}


/***************************************************
函数名: void nmotor_output(uint8_t _speed_ctrl_mode)
说明:	最终PWM控制输出
入口:	uint8_t _speed_ctrl_mode-速度控制方式,0:调试模式;1:普通速度控制;2:自平衡速度控制
出口:	无
备注:	无
作者:	无名创新
****************************************************/
void nmotor_output(uint8_t _speed_ctrl_mode)
{

	if(_speed_ctrl_mode==0)//仅在初始调试电机运动方向时使用
	{
		left_pwm =motion_ctrl_pwm;
		right_pwm=motion_ctrl_pwm;
	}
	else if(_speed_ctrl_mode==1)//两轮速度单独控制
	{
		right_pwm=speed_output[1];
		left_pwm =speed_output[0];
	}
	if(raspi_ctrl_procedure.instruture == Raspi_Ctrl_OPen_Loop_Output_Pwm + Instruction_Base_Address)
	{
		right_pwm = raspi_ctrl_procedure.right_pwm;
		left_pwm = raspi_ctrl_procedure.left_pwm;

	}
	else
	{
		right_pwm = 0;
		left_pwm = 0;	
	}
	
	if(trackless_output.unlock_flag==LOCK)   // 如果是锁着的话
	{
		left_pwm =0;
		right_pwm=0;	
	}
	
	left_pwm =constrain_float(left_pwm,-motor_max_default,motor_max_default);
	right_pwm=constrain_float(right_pwm,-motor_max_default,motor_max_default);
	
	//右边电机输出
	if(trackless_motor.right_motion_dir_config==0)
	{
		if(right_pwm>=0)
		{
			PWMPulseWidthSet(PWM0_BASE,PWM_OUT_0,0);			 				//PH0——M0PWM0 
			PWMPulseWidthSet(PWM0_BASE,PWM_OUT_1,right_pwm);			//PH1——M0PWM1
		}
		else
		{
			PWMPulseWidthSet(PWM0_BASE,PWM_OUT_0,ABS(right_pwm));	//PH0——M0PWM0 
			PWMPulseWidthSet(PWM0_BASE,PWM_OUT_1,0);			       	//PH1——M0PWM1 	
		}
	}
	else
	{
		if(right_pwm>=0)
		{
			PWMPulseWidthSet(PWM0_BASE,PWM_OUT_0,right_pwm);//PH0——M0PWM0 
			PWMPulseWidthSet(PWM0_BASE,PWM_OUT_1,0);			  //PH1——M0PWM1
		}
		else
		{
			PWMPulseWidthSet(PWM0_BASE,PWM_OUT_0,0);	           //PH0——M0PWM0 
			PWMPulseWidthSet(PWM0_BASE,PWM_OUT_1,ABS(right_pwm));//PH1——M0PWM1 	
		}
	}
	//左边电机输出
	if(trackless_motor.left_motion_dir_config==0)
	{	
		if(left_pwm>=0)
		{
			PWMPulseWidthSet(PWM0_BASE,PWM_OUT_2,left_pwm);  			//PH2——M0PWM2 
			PWMPulseWidthSet(PWM0_BASE,PWM_OUT_3,0);				 			//PH3——M0PWM3  		
		}
		else
		{
			PWMPulseWidthSet(PWM0_BASE,PWM_OUT_2,0); 							//PH2——M0PWM2 
			PWMPulseWidthSet(PWM0_BASE,PWM_OUT_3,ABS(left_pwm));	//PH3——M0PWM3 
		}
	}
	else
	{
		if(left_pwm>=0)
		{
			PWMPulseWidthSet(PWM0_BASE,PWM_OUT_2,0);  			//PH2——M0PWM2 
			PWMPulseWidthSet(PWM0_BASE,PWM_OUT_3,left_pwm);	//PH3——M0PWM3  		
		}
		else
		{
			PWMPulseWidthSet(PWM0_BASE,PWM_OUT_2,ABS(left_pwm));//PH2——M0PWM2 
			PWMPulseWidthSet(PWM0_BASE,PWM_OUT_3,0);	          //PH3——M0PWM3 
		}	
	}
}

/**
 * @brief 开环输出pwm
*/
void Open_Loop_Motor_Output(int16_t left_pwm, int16_t right_pwm)
{


	left_pwm =constrain_float(left_pwm,-motor_max_default,motor_max_default);
	right_pwm=constrain_float(right_pwm,-motor_max_default,motor_max_default);
	
	//右边电机输出
	if(trackless_motor.right_motion_dir_config==0)
	{
		if(right_pwm>=0)
		{
			PWMPulseWidthSet(PWM0_BASE,PWM_OUT_0,0);			 				//PH0——M0PWM0 
			PWMPulseWidthSet(PWM0_BASE,PWM_OUT_1,right_pwm);			//PH1——M0PWM1
		}
		else
		{
			PWMPulseWidthSet(PWM0_BASE,PWM_OUT_0,ABS(right_pwm));	//PH0——M0PWM0 
			PWMPulseWidthSet(PWM0_BASE,PWM_OUT_1,0);			       	//PH1——M0PWM1 	
		}
	}
	else
	{
		if(right_pwm>=0)
		{
			PWMPulseWidthSet(PWM0_BASE,PWM_OUT_0,right_pwm);//PH0——M0PWM0 
			PWMPulseWidthSet(PWM0_BASE,PWM_OUT_1,0);			  //PH1——M0PWM1
		}
		else
		{
			PWMPulseWidthSet(PWM0_BASE,PWM_OUT_0,0);	           //PH0——M0PWM0 
			PWMPulseWidthSet(PWM0_BASE,PWM_OUT_1,ABS(right_pwm));//PH1——M0PWM1 	
		}
	}
	//左边电机输出
	if(trackless_motor.left_motion_dir_config==0)
	{	
		if(left_pwm>=0)
		{
			PWMPulseWidthSet(PWM0_BASE,PWM_OUT_2,left_pwm);  			//PH2——M0PWM2 
			PWMPulseWidthSet(PWM0_BASE,PWM_OUT_3,0);				 			//PH3——M0PWM3  		
		}
		else
		{
			PWMPulseWidthSet(PWM0_BASE,PWM_OUT_2,0); 							//PH2——M0PWM2 
			PWMPulseWidthSet(PWM0_BASE,PWM_OUT_3,ABS(left_pwm));	//PH3——M0PWM3 
		}
	}
	else
	{
		if(left_pwm>=0)
		{
			PWMPulseWidthSet(PWM0_BASE,PWM_OUT_2,0);  			//PH2——M0PWM2 
			PWMPulseWidthSet(PWM0_BASE,PWM_OUT_3,left_pwm);	//PH3——M0PWM3  		
		}
		else
		{
			PWMPulseWidthSet(PWM0_BASE,PWM_OUT_2,ABS(left_pwm));//PH2——M0PWM2 
			PWMPulseWidthSet(PWM0_BASE,PWM_OUT_3,0);	          //PH3——M0PWM3 
		}	
	}
}







