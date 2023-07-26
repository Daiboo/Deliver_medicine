#include "Headfile.h"
#include "Buzzer.h"



_laser_light beep;

/***************************************
函数名:	void Buzzer_Init(void)
说明: GPIO初始化
入口:	无
出口:	无
备注:	无
作者:	无名创新
***************************************/
void Buzzer_Init(void)
{
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOJ);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
	
	GPIOPinTypeGPIOOutput(GPIO_PORTJ_BASE, GPIO_PIN_2);   // pj2 -- 蜂鸣器驱动电路
	
	GPIOPadConfigSet(GPIO_PORTJ_BASE,GPIO_PIN_2,GPIO_STRENGTH_4MA,GPIO_PIN_TYPE_STD);  // 模式配置 推挽输出
	GPIOPinWrite(GPIO_PORTJ_BASE,GPIO_PIN_2,0);
	
	GPIOPinTypeGPIOOutput(GPIO_PORTE_BASE, GPIO_PIN_2);   // pe2 -- 程序控制对外电源输出 IO控制口
	GPIOPadConfigSet(GPIO_PORTE_BASE,GPIO_PIN_2,GPIO_STRENGTH_4MA,GPIO_PIN_TYPE_STD);	
	GPIOPinWrite(GPIO_PORTE_BASE,GPIO_PIN_2,0);
	
	// 蜂鸣器
	beep.port=GPIO_PORTJ_BASE;
	beep.pin=GPIO_PIN_2;
	beep.period=20;//20*5ms
	beep.light_on_percent=0.5f;   // 占空比
	beep.reset=0;	
}

/***************************************
函数名:	laser_light_work(_laser_light *light)
说明: gpio驱动状态机
入口:	_laser_light *light-gpio控制结构体   原理是模拟pwm波形
出口:	无
备注:	无
作者:	无名创新
***************************************/
void laser_light_work(_laser_light *light)
{
	if(light->reset==1)
	{
		light->reset=0;
		light->cnt=0;
		light->times_cnt=0;//点亮次数计数器清零
		light->end=0;
	}
	
	if(light->times_cnt==light->times)
	{
		light->end=1;
		return;
	}

	light->cnt++;
	if(light->cnt<=light->period * light->light_on_percent)  // 小于一半
	{
		GPIOPinWrite(light->port,light->pin,light->pin);  // 1
	}
	else if(light->cnt<light->period)     // 大于一半小于整个
	{
		GPIOPinWrite(light->port,light->pin,0);
	}
	else//完成点亮一次   如果等于period
	{
		GPIOPinWrite(light->port,light->pin,0);
		light->cnt=0;
		light->times_cnt++;
	}
}

