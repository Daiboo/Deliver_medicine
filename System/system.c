#include "system.h"
#include "headfile.h"

static volatile uint32_t counter;
volatile uint32_t systick_clk,systick_period=0,systick_cnt=0;


static void SycTickHandler(void) {counter++;}  // 
/***************************************
函数名:	void Systick_Init(void)
说明: 滴答定时器初始化
入口:	无
出口:	无
备注:	无
作者:	无名创新
***************************************/
void Systick_Init(void) 
{
	systick_clk=SysCtlClockGet();//主频
  SysTickPeriodSet(systick_clk/500UL);//2ms——1000 for milliseconds & 1000000 for microseconds 
	SysTickIntRegister(SycTickHandler);//滴答定时器中断服务函数注册
  SysTickIntEnable();//滴答中断使能
  SysTickEnable();//滴答计数使能
	systick_period=SysTickPeriodGet();//获取最大计数值
}

/***************************************
函数名:	void System_Init(void)
说明: 系统时钟初始化
入口:	无
出口:	无
备注:	无
作者:	无名创新
***************************************/
void System_Init(void)
{
  SysCtlClockSet(SYSCTL_SYSDIV_2_5 | SYSCTL_USE_PLL | SYSCTL_XTAL_16MHZ |SYSCTL_OSC_MAIN);//系统时钟频率400M/2/2.5=80M
  FPUEnable();//使能浮点单元
  FPULazyStackingEnable();//浮点延迟堆栈,减少中断响应延迟
  Systick_Init();//滴答定时器初始化
  IntMasterEnable();//全局中断使能
  IntPriorityGroupingSet(3);//设置中断组	
}	

/***************************************
函数名:	void delay(uint32_t ms)
说明: 延迟ms
入口:	uint32_t ms-延时ms数
出口:	无
备注:	无
作者:	无名创新
***************************************/
void delay(uint32_t ms) 
{
  delayMicroseconds(ms * 1000UL);
}

/***************************************
函数名:	void delayMicroseconds(uint32_t us)
说明: 延迟us
入口:	uint32_t us-延时us数
出口:	无
备注:	无
作者:	无名创新
***************************************/
void delayMicroseconds(uint32_t us) 
{
  uint32_t start = micros();
  while ((int32_t)(micros() - start) < us) {
    // Do nothing
  };
}

/***************************************
函数名:	uint32_t millis(void)
说明: 获取系统运行时间ms
入口:	无
出口:	uint32_t 系统已运行时间ms
备注:	无
作者:	无名创新
***************************************/
uint32_t millis(void) 
{
	return micros() / 1000UL;
}

/***************************************
函数名:	uint32_t micros(void)
说明: 获取系统运行时间us
入口:	无
出口:	uint32_t 系统已运行时间us
备注:	无
作者:	无名创新
***************************************/
uint32_t micros(void) {
	systick_cnt=SysTickValueGet();
	return (counter*2000 + 2000*(systick_period-systick_cnt)/systick_period);
}

/***************************************
函数名:	void Delay_Ms(uint32_t x)
说明: 延迟ms
入口:	uint32_t x-延时ms数
出口:	无
备注:	无
作者:	无名创新
***************************************/
void Delay_Ms(uint32_t x)
{
  delay(x);
}

/***************************************
函数名:	void delay_ms(uint32_t x)
说明: 延迟ms
入口:	uint32_t x-延时ms数
出口:	无
备注:	无
作者:	无名创新
***************************************/
void delay_ms(uint32_t x)
{
  Delay_Ms(x);
}

/***************************************
函数名:	void delay_us(uint32_t x)
说明: 延迟us
入口:	uint32_t x-延时us数
出口:	无
备注:	无
作者:	无名创新
***************************************/
void delay_us(uint32_t x)
{
  delayMicroseconds(x);
}

/***************************************
函数名:	void Delay_Us(uint32_t x)
说明: 延迟us
入口:	uint32_t x-延时us数
出口:	无
备注:	无
作者:	无名创新
***************************************/
void Delay_Us(uint32_t x) 
{
  delayMicroseconds(x);
}


/***************************************
函数名:	void get_systime(systime *sys)
说明: 获取系统运行时间、任务调度周期等
入口:	systime *sys-任务运行时间结构体指针
出口:	无
备注:	无
作者:	无名创新
***************************************/
void get_systime(systime *sys)
{
  sys->last_time=sys->current_time;
  sys->current_time=micros()/1000.0f;//单位ms
  sys->period=sys->current_time-sys->last_time;
  sys->period_int=(uint16_t)(sys->period+0.5f);//四舍五入
}
