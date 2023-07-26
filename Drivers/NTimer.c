#include "Headfile.h"
#include "NTimer.h"


systime timer_t0a, timer_t1a, timer_t2a, timer_t3a;

// ****************任务*****************
extern void duty_10hz(void);
extern void duty_100hz(void);
extern void duty_200hz(void);
extern void duty_1000hz(void);



// ***************定时中断handler************

void TIMER0A_Handler(void);
void TIMER1A_Handler(void);
void TIMER2A_Handler(void);
void TIMER3A_Handler(void);


/***************************************
函数名:	void Time0A_init(void)
说明: timer0A定时器初始化
入口:	无
出口:	无
备注:	无
作者:	无名创新
***************************************/
void Time0A_Init(void)  // 系统调度定时器初始化
{
	SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);  // 定时器使能
	TimerConfigure(TIMER0_BASE,TIMER_CFG_PERIODIC);  // 32位周期定时器
	TimerLoadSet(TIMER0_BASE,TIMER_A,SysCtlClockGet()/200);  // 设定装载值，（80M / 200） * 1 / 80M = 5ms
	IntEnable(INT_TIMER0A); 						// 总中断使能
	TimerIntEnable(TIMER0_BASE,TIMER_TIMA_TIMEOUT);  // 中断输出，设置模式
	TimerIntRegister(TIMER0_BASE, TIMER_A, TIMER0A_Handler);  // 中断函数注册
	
	TimerEnable(TIMER0_BASE, TIMER_A);   // 定时器使能开始计数
	IntPrioritySet(INT_TIMER0A,USER_INT7);
	
}

/***************************************
函数名:	void Time1A_init(void)
说明: timer1A定时器初始化
入口:	无
出口:	无
备注:	无
作者:	无名创新
***************************************/
void Time1A_Init(void)
{
	SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER1);//定时器1使能				
	TimerConfigure(TIMER1_BASE,TIMER_CFG_PERIODIC_UP);//32位周期定时器
	TimerLoadSet(TIMER1_BASE,TIMER_A,SysCtlClockGet()/1000);//设定装载值,（80M/1000*1/80M=1ms				
	IntEnable(INT_TIMER1A);//定时器1中断使能				
	TimerIntEnable(TIMER1_BASE,TIMER_TIMA_TIMEOUT); //中断输出, 设置模式;
	TimerIntRegister(TIMER1_BASE,TIMER_A,TIMER1A_Handler);//中断函数注册
	//  IntMasterEnable();			
	TimerEnable(TIMER1_BASE,TIMER_A); //定时器使能开始计数	
	IntPrioritySet(INT_TIMER1A,USER_INT5);
}


/***************************************
函数名:	void Time2A_init(void)
说明: timer2A定时器初始化
入口:	无
出口:	无
备注:	无
作者:	无名创新
***************************************/
void Time2A_Init(void)//系统调度定时器初始化
{
	SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER2);							//定时器0使能				
	TimerConfigure(TIMER2_BASE,TIMER_CFG_PERIODIC);						//32位周期定时器				
	TimerLoadSet(TIMER2_BASE,TIMER_A,SysCtlClockGet()/100);		//设定装载值,（80M/100）*1/80M=10ms				
	IntEnable(INT_TIMER2A);																		//总中断使能				
	TimerIntEnable(TIMER2_BASE,TIMER_TIMA_TIMEOUT); 					//中断输出, 设置模式;			
	TimerIntRegister(TIMER2_BASE,TIMER_A,TIMER2A_Handler);		//中断函数注册
	//  IntMasterEnable();			
	TimerEnable(TIMER2_BASE,TIMER_A); 												//定时器使能开始计数
	IntPrioritySet(INT_TIMER2A,USER_INT7);
}


/***************************************
函数名:	void Time3A_init(uint32_t x)
说明: timer3A定时器初始化
入口:	无
出口:	无
备注:	无
作者:	无名创新
***************************************/
void Time3A_init(void)//系统调度定时器初始化
{
	SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER3);							//定时器0使能				
	TimerConfigure(TIMER3_BASE,TIMER_CFG_PERIODIC);						//32位周期定时器				
	TimerLoadSet(TIMER3_BASE,TIMER_A,SysCtlClockGet()/10);		//设定装载值,（80M/10）*1/80M=100ms				
	IntEnable(INT_TIMER3A);																		//总中断使能				
	TimerIntEnable(TIMER3_BASE,TIMER_TIMA_TIMEOUT); 					//中断输出, 设置模式;			
	TimerIntRegister(TIMER3_BASE,TIMER_A,TIMER3A_Handler);		//中断函数注册
	//  IntMasterEnable();			
	TimerEnable(TIMER3_BASE,TIMER_A); 												//定时器使能开始计数
	IntPrioritySet(INT_TIMER3A,USER_INT7);
}


/***************************************
函数名:	void NTimer_Init(void)
说明: 系统定时器初始化
入口:	无
出口:	无
备注:	无
作者:	无名创新
***************************************/
void NTimer_Init(void)
{
	IntMasterDisable();   // 中断失能
	Time0A_Init();     // 对应200hz
	Time1A_Init();
//	Time2A_Init();
//	Time3A_init();
	IntMasterEnable();    // 中断使能
}

/***************************************
函数名:	void TIMER0A_Handler(void)
说明: timer0A定时器中断服务函数
入口:	无
出口:	无
备注:	无
作者:	无名创新
***************************************/
void TIMER0A_Handler(void)
{
	get_systime(&timer_t0a);
	duty_200hz();
	TimerIntClear(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
}

/***************************************
函数名:	void TIMER1A_Handler(void)
说明: timer1A定时器中断服务函数
入口:	无
出口:	无
备注:	无
作者:	无名创新
***************************************/
void TIMER1A_Handler(void)  // 温控中断函数
{
	get_systime(&timer_t1a);
	duty_1000hz();
	TimerIntClear(TIMER1_BASE, TIMER_TIMA_TIMEOUT);
	
}

/***************************************
函数名:	void TIMER2A_Handler(void)
说明: timer2A定时器中断服务函数
入口:	无
出口:	无
备注:	无
作者:	无名创新
***************************************/
void TIMER2A_Handler(void)      // 地面站数据发送中断函数
{
	get_systime(&timer_t2a);
	duty_100hz();
	TimerIntClear(TIMER2_BASE, TIMER_TIMA_TIMEOUT);
	
}

/***************************************
函数名:	void TIMER3A_Handler(void)
说明: timer3A定时器中断服务函数
入口:	无
出口:	无
备注:	无
作者:	无名创新
***************************************/
void TIMER3A_Handler(void)   // 地面站数据发送中断函数
{
	get_systime(&timer_t3a);
	duty_10hz();
	TimerIntClear(TIMER3_BASE, TIMER_TIMA_TIMEOUT);
}



/***************************************
函数名:	void FaultISR(void)
说明: 硬件故障中断服务函数
入口:	无
出口:	无
备注:	无
作者:	无名创新
***************************************/

void FaultISR(void)
{
//	PWM_Output(0,0,0,0);//将电调控制信号给0
//	GPIOPinWrite(heater_port,heater_pin,0);//将IMU温度控制信号失能
//	SysCtlReset();
	
}
