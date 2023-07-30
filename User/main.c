#include "Headfile.h"


int main()
{
	System_Init();  // 系统时钟初始化
	EEPROM_Init();	//EEPROM初始化
	OLED_Init();
	ADC_Init();      // 采集电压

	I2C1_Init();     // I2C1初始化:ICM20608,SPL06
	rgb_init();
	Buzzer_Init();    // 初始化蜂鸣器和对外输出电源控制io
	

	
	ctrl_params_init();			//控制参数初始化	
	trackless_params_init();	//硬件配置初始化
	
	
	
	PWM0_Init();  // PWM0初始化M0P0、M0P1、M0P2、M0P3
	PWM1_Init();  // PWM1初始化M1P0、M1P1、M1P2、M1P3

	rangefinder_init();			// 测距传感器串口
	UART3_Init(115200);  		// 树莓派通信串口
	UART7_Init(256000);			// OPENMV视觉串口初始化
	


	simulation_pwm_init();   // 模拟PWM初始化  用于加热imu
	
	
	ICM206xx_Init();				//加速度计/陀螺仪初始化
	
	Encoder_Init();					//编码器资源初始化
	Button_Init();					//板载按键初始化
	NTimer_Init();
	
	page_number = 8;
	sdk_work_mode = Deliver_Medicine;
	// sdk_work_mode = Car_Stop;
	Tidata_Tosend_OpenMV(Tracking_task);
	while(1)
	{
		screen_display();//屏幕显示
		
		
	}
}


/***************************************
函数名:	void duty_200hz(void)
说明: 200hz实时任务函数
入口:	无
出口:	无
备注:	无
作者:	无名创新
***************************************/
void duty_200hz(void)    // 每5ms执行一次
{
	get_wheel_speed();					   //获取轮胎转速
	Raspi_Ctrl_Instruction_Dispatch();
	sdk_duty_run();					  		 //SDK总任务控制，调参
	nmotor_output(speed_ctrl_mode);          //控制器输出，主要是输出pwm的
	// rangefinder_statemachine();		 //超声波传感器数据获取
	adc_sample_trigger();					 //ADC数据获取
	imu_data_sampling();					 //加速度计、陀螺仪数据获采集
	trackless_ahrs_update();			 //ahrs姿态更新
	imu_temperature_ctrl();				 //传感器恒温控制
	read_button_state_all();			 //按键状态检测
	battery_voltage_detection();	 //电池电压检测

	bling_working(0);				// 状态指示灯状态机
	laser_light_work(&beep);			 //蜂鸣器状态机
}

/***************************************
函数名:	void duty_1000hz(void)
说明: 1000hz实时任务函数
入口:	无
出口:	无
备注:	无
作者:	无名创新
***************************************/

void duty_1000hz(void)
{
//	if(sdk_work_mode==15)
//	{
//		gpio_input_check_channel_12_with_handle();//检测12路灰度灰度管状态,带赛道信息处理
//	}
//	else
//	{
//		gpio_input_check_channel_12();//检测12路灰度灰度管状态
//	}
	// gpio_input_check_from_vision();//openmv机器视觉信息获取
	simulation_pwm_output();//模拟pwm输出
}

/***************************************
函数名:	void duty_100hz(void)
说明: 100hz实时任务函数
入口:	无
出口:	无
备注:	无
作者:	无名创新
***************************************/

void duty_100hz(void)
{

}

/***************************************
函数名:	void duty_10hz(void)
说明: 10hz实时任务函数
入口:	无
出口:	无
备注:	无
作者:	无名创新
***************************************/

void duty_10hz(void)
{

}
