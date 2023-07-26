#include "Headfile.h"
#include "user.h"
#include "Ultrasonic.h"

uint8_t  com5_rx_buf[4]={0};
uint16_t com5_rx_cnt=0;
_rangefinder rangefinder;

/***************************************
函数名:	void us100_start(void)
说明: us100测距触发指令
入口:	无
出口:	无
备注:	无
作者:	无名创新
***************************************/
void us100_start(void)
{
	UARTCharPut(UART5_BASE,0x55);
}

/***************************************
函数名:	float us100_get_distance(uint8 MSB,uint8 LSB)
说明: 超声波回传的两个字节换算成实际距离
入口:	uint8 MSB-高字节
			uint8 LSB-低字节
出口:	float 返回距离值
备注:	无
作者:	无名创新
***************************************/
float us100_get_distance(uint8_t MSB,uint8_t LSB)
{
  return (256*(MSB)+(LSB))/10.0f;//单位cm
}

/***************************************
函数名:	float us100_get_temperature(uint8_t data)
说明: 获取温度数据
入口:	uint8 data-温度字节
出口:	float 返回温度值
备注:	无
作者:	无名创新
***************************************/
float us100_get_temperature(uint8_t data)
{
  return (data-45)/1.0;//℃
}

/***************************************
函数名:	void us100_statemachine(void)
说明: us100测量状态机
入口:	无
出口:	无
备注:	无
作者:	无名创新
***************************************/
void us100_statemachine(void)
{
	static uint16_t us100_cnt=0;
	us100_cnt++;
	if(us100_cnt>=20)
	{		
		us100_cnt=0;
		
		rangefinder.pre_last_distance=rangefinder.last_distance;//上上次高度
		rangefinder.last_distance=rangefinder.distance;//上次高度
		rangefinder.distance=us100_get_distance(com5_rx_buf[0],com5_rx_buf[1]);

		rangefinder.last_vel=rangefinder.vel;
		rangefinder.vel=(rangefinder.distance-rangefinder.last_distance)/0.1f;
		rangefinder.acc=(rangefinder.vel-rangefinder.last_vel)/0.1f;
		
		com5_rx_cnt=0;
		us100_start();
	}
}

/***************************************
函数名:	void nHCSR04_Handler(void)
说明: 超声波采用外部中断方式接收时的中断服务函数
入口:	无
出口:	无
备注:	无
作者:	无名创新
***************************************/
void nHCSR04_Handler(void)//中断程序
{
	GPIOIntClear(GPIO_PORTE_BASE,GPIO_PIN_4);//清除中断标志
	if(rangefinder.cnt==0)//记录上升沿时刻时间
	{
		rangefinder.rf_start_time=micros();//单位us
		GPIOIntTypeSet(GPIO_PORTE_BASE, GPIO_PIN_4 , GPIO_FALLING_EDGE);//中断触发类型为下降沿触发
		rangefinder.cnt++;
	}
	else if(rangefinder.cnt==1)//记录下降沿时刻时间
	{
		rangefinder.rf_end_time=micros();//单位us
		rangefinder.rf_delta=rangefinder.rf_end_time-rangefinder.rf_start_time;
		rangefinder.rf_update_flag=1;
		GPIOIntDisable(GPIO_PORTE_BASE, GPIO_PIN_4);//失能PE4中断
	}
}

/***************************************
函数名:	void hcsr04_init(void)
说明: 超声波芯片资源初始化
入口:	无
出口:	无
备注:	IO口触发+外部中断解析
作者:	无名创新
***************************************/
void hcsr04_init(void)
{
  SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);  //使能GPIO外设
  GPIOIntRegister(GPIO_PORTE_BASE, nHCSR04_Handler);    //GPIO注册中断
  GPIOPinTypeGPIOInput(GPIO_PORTE_BASE, GPIO_PIN_4);    //PE4作为中断输入源
  GPIOPadConfigSet(GPIO_PORTE_BASE,GPIO_PIN_4,GPIO_STRENGTH_2MA,GPIO_PIN_TYPE_STD_WPU); //配置PC3为上拉
  GPIOIntTypeSet(GPIO_PORTE_BASE, GPIO_PIN_4 , GPIO_RISING_EDGE);   //中断触发类型为下降沿触发
  GPIOIntDisable(GPIO_PORTE_BASE, GPIO_PIN_4);  //失能PE4中断
  IntPrioritySet(INT_GPIOE,USER_INT1);
	
  SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
  GPIOPinTypeGPIOOutput(GPIO_PORTE_BASE, GPIO_PIN_5);
}

/***************************************
函数名:	void hcsr04_statemachine(void)
说明: 超声波采集状态机
入口:	无
出口:	无
备注:	IO口触发
作者:	无名创新
***************************************/
void hcsr04_statemachine(void)
{
	static uint16_t cnt=0;
  cnt++;
	if(cnt>=10)
	{
		cnt=0;
		if(rangefinder.rf_update_flag==1)
		{
			rangefinder.pre_last_distance=rangefinder.last_distance;//上上次高度
			rangefinder.last_distance=rangefinder.distance;//上次高度
			rangefinder.distance=rangefinder.rf_delta*340/20000.0f;//转化成cm
			rangefinder.last_vel=rangefinder.vel;
			rangefinder.vel=(rangefinder.distance-rangefinder.last_distance)/0.1f;
			rangefinder.acc=(rangefinder.vel-rangefinder.last_vel)/0.1f;		
		}
		
		GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_5, GPIO_PIN_5);
		delay_us(20);
		GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_5, 0);
		rangefinder.cnt=0;
		GPIOIntTypeSet(GPIO_PORTE_BASE, GPIO_PIN_4 , GPIO_RISING_EDGE);//中断触发类型为上升沿触发
		GPIOIntEnable(GPIO_PORTE_BASE, GPIO_PIN_4);//使能PE4中断
	}
}

/***************************************
函数名:	void rangefinder_init(void)
说明: 测距传感器初始化
入口:	无
出口:	无
备注:	无
作者:	无名创新
***************************************/
void rangefinder_init(void)
{
	float tmp_sensor_type=0;
	ReadFlashParameterOne(RANGEFINDER_TYPE,&tmp_sensor_type);
	if(isnan(tmp_sensor_type)==0) 	rangefinder.sensor_type=tmp_sensor_type;				
	else rangefinder.sensor_type=rangefinder_type_default;
	rangefinder.sensor_init_type=rangefinder.sensor_type;
	switch(rangefinder.sensor_type)
	{
		case 0:hcsr04_init();break;
	  case 1:UART5_Init(9600);break;
		default:UART5_Init(9600);
	}
	
	
}	


/***************************************
函数名:	void rangefinder_statemachine(void)
说明: 测距采集状态机
入口:	无
出口:	无
备注:	无
作者:	无名创新
***************************************/
void rangefinder_statemachine(void)
{
	if(rangefinder.sensor_init_type!=rangefinder.sensor_type)//初始化时传感器类型与当前设置传感器类型不一致
	{
		return;
		//重启复位才生效
	}
	switch(rangefinder.sensor_type)
	{
		case 0:hcsr04_statemachine();break;
	  case 1:us100_statemachine();break;
		default:us100_statemachine();
	}
}	
