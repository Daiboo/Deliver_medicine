#include "headfile.h"
#include "adc.h"
#include "nadc.h"
#include "user.h"

uint32_t adc_value[8];
void nADC0_Handler(void);

/***************************************
函数名:	void ADC_Init(void)
说明: ADC初始化配置
入口:	无
出口:	无
备注:	无
作者:	无名创新
***************************************/
void ADC_Init(void)  
{    
  SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);// Enable the ADC1 module.
  while(!SysCtlPeripheralReady(SYSCTL_PERIPH_ADC0));// Wait for the ADC1 module to be ready.	
  SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
	
  GPIOPinTypeADC(GPIO_PORTE_BASE, GPIO_PIN_3);//AIN0
//	GPIOPinTypeADC(GPIO_PORTE_BASE, GPIO_PIN_2);//AIN1
	
//	GPIOPinTypeADC(GPIO_PORTD_BASE, GPIO_PIN_0);//AIN15
//	GPIOPinTypeADC(GPIO_PORTD_BASE, GPIO_PIN_1);//AIN14
//	GPIOPinTypeADC(GPIO_PORTD_BASE, GPIO_PIN_2);//AIN13
//	GPIOPinTypeADC(GPIO_PORTD_BASE, GPIO_PIN_3);//AIN12
	
  //ADCReferenceSet(ADC0_BASE, ADC_REF_EXT_3V);
  //Enable the first sample sequencer to capture the value of channel 0 when
  //the processor trigger occurs.  
  ADCSequenceConfigure(ADC0_BASE,0,ADC_TRIGGER_PROCESSOR, 0);
  //ADCHardwareOversampleConfigure(ADC0_BASE, 8);	
	ADCSequenceStepConfigure(ADC0_BASE, 0, 0, ADC_CTL_CH0 | ADC_CTL_END | ADC_CTL_IE);
	//ADCSequenceStepConfigure(ADC0_BASE, 0, 1, ADC_CTL_CH1);
	//ADCSequenceStepConfigure(ADC0_BASE, 0, 2, ADC_CTL_CH15);
	//ADCSequenceStepConfigure(ADC0_BASE, 0, 3, ADC_CTL_CH14);
	//ADCSequenceStepConfigure(ADC0_BASE, 0, 4, ADC_CTL_CH13);
	//ADCSequenceStepConfigure(ADC0_BASE, 0, 5, ADC_CTL_CH12	| ADC_CTL_END | ADC_CTL_IE);
	
	ADCIntClear(ADC0_BASE, 0);
	ADCSequenceEnable(ADC0_BASE, 0);    
  ADCIntEnable(ADC0_BASE, 0); 
  //中断触发方式设置ADC_INT_SS0、ADC_INT_DMA_SS0、ADC_INT_DCON_SS0
	ADCIntEnableEx(ADC0_BASE,ADC_INT_SS0);//分别代表普通序列触发、DMA触发和数字比较器触发
	IntEnable(INT_ADC0SS0); //使能ADC采样序列中断	
	ADCIntRegister(ADC0_BASE,0,nADC0_Handler);		//中断函数注册
	IntPrioritySet(INT_ADC0SS0, USER_INT7);	
} 

/***************************************
函数名:	void nADC0_Handler(void)
说明: ADC中断服务函数
入口:	无
出口:	无
备注:	无
作者:	无名创新
***************************************/
void nADC0_Handler(void)
{
	ADCIntClear(ADC0_BASE, 0);// 清除ADC中断标志。
	//while(!ADCIntStatus(ADC0_BASE, 0, false));//等待采集结束
	ADCSequenceDataGet(ADC0_BASE, 0, adc_value);// 读取ADC值
}

/***************************************
函数名:	void adc_sample_trigger(void)
说明: ADC采样触发
入口:	无
出口:	无
备注:	无
作者:	无名创新
***************************************/
void adc_sample_trigger(void)
{
	static uint16_t cnt=0;
	cnt++;
	if(cnt<=2) return;
	cnt=0;
	ADCProcessorTrigger(ADC0_BASE, 0);
}



low_voltage vbat={
	.enable=no_voltage_enable_default,
	.value=0,
	.upper=no_voltage_upper_default,
	.lower=no_voltage_lower_default
};

/***************************************
函数名:	void get_battery_voltage(void)
说明: ADC采样触发
入口:	无
出口:	无
备注:	测量电池电压，飞控默认分压比为11，故测量电压不要超过
3.3V*11=36.6V，若想测更大的电压，调整板子上分压电阻阻值即可
作者:	无名创新
***************************************/
float get_battery_voltage(void)//ADC获取   
{
	static float value_filter;
	value_filter=0.9f*value_filter+0.1f*adc_value[0]*36.3f/4095.0f;	
	vbat.value=value_filter;
  return vbat.value;	
}

/***************************************
函数名:	void battery_voltage_detection(void)
说明: 电池电压监测
入口:	无
出口:	无
备注:	需要设定电压阈值
作者:	无名创新
***************************************/
void battery_voltage_detection(void)
{
	static uint16_t _cnt=0;
	_cnt++;
	if(_cnt>=200)//每1S检测一次
	{
		_cnt=0;
		if(vbat.value<vbat.upper&&vbat.value>vbat.lower)	 vbat.low_vbat_cnt++;
		else vbat.low_vbat_cnt/=2;
		if(vbat.low_vbat_cnt>=10)//持续10s满足
		{
			if(vbat.enable==1)
			{
				beep.period=200;//200*5ms
				beep.light_on_percent=0.25f;
				beep.reset=1;	
				beep.times=5;//闪烁5次
			}
			vbat.low_vbat_cnt=0;//清0待下一周期检测			
		}			
	}
}
