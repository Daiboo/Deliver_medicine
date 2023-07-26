#include "Headfile.h"
#include "nbutton.h"

rc_buttton _button;

/***************************************
函数名:	void Button_Init(void)
说明: 板载按键初始化
入口:	无
出口:	无
备注:	无
作者:	无名创新
***************************************/
void Button_Init(void)
{
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOJ);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOK);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
	
	GPIODirModeSet(keydown_port,keydown_pin,GPIO_DIR_MODE_IN);//SW1
	GPIODirModeSet(keyup_port  ,keyup_pin,GPIO_DIR_MODE_IN);//SW2
	GPIOPadConfigSet(keydown_port,keydown_pin,GPIO_STRENGTH_2MA,GPIO_PIN_TYPE_STD_WPU);
	GPIOPadConfigSet(keyup_port  ,keyup_pin,GPIO_STRENGTH_2MA,GPIO_PIN_TYPE_STD_WPU);
	
	/******************************************************************************************/
	GPIODirModeSet(GPIO_PORTK_BASE, GPIO_PIN_0,GPIO_DIR_MODE_IN);
	GPIODirModeSet(GPIO_PORTK_BASE, GPIO_PIN_1,GPIO_DIR_MODE_IN);
	GPIODirModeSet(GPIO_PORTK_BASE, GPIO_PIN_2,GPIO_DIR_MODE_IN);
	GPIODirModeSet(GPIO_PORTK_BASE, GPIO_PIN_3,GPIO_DIR_MODE_IN);
	GPIODirModeSet(GPIO_PORTF_BASE, GPIO_PIN_7,GPIO_DIR_MODE_IN);
	
	GPIOPadConfigSet(GPIO_PORTK_BASE,GPIO_PIN_0,GPIO_STRENGTH_2MA,GPIO_PIN_TYPE_STD_WPU);
	GPIOPadConfigSet(GPIO_PORTK_BASE,GPIO_PIN_1,GPIO_STRENGTH_2MA,GPIO_PIN_TYPE_STD_WPU);
	GPIOPadConfigSet(GPIO_PORTK_BASE,GPIO_PIN_2,GPIO_STRENGTH_2MA,GPIO_PIN_TYPE_STD_WPU);
	GPIOPadConfigSet(GPIO_PORTK_BASE,GPIO_PIN_3,GPIO_STRENGTH_2MA,GPIO_PIN_TYPE_STD_WPU);
	GPIOPadConfigSet(GPIO_PORTF_BASE,GPIO_PIN_7,GPIO_STRENGTH_2MA,GPIO_PIN_TYPE_STD_WPU);
	/******************************************************************************************/
	_button.state[UP].port=keyup_port;
	_button.state[UP].pin=keyup_pin;
	_button.state[UP].value=1;
	_button.state[UP].last_value=1;

	_button.state[DOWN].port=keydown_port;
	_button.state[DOWN].pin=keydown_pin;
	_button.state[DOWN].value=1;
	_button.state[DOWN].last_value=1;	
	/******************************************************************************************/
	_button.state[UP_3D].port=GPIO_PORTF_BASE;
	_button.state[UP_3D].pin=GPIO_PIN_7;
	_button.state[UP_3D].value=1;
	_button.state[UP_3D].last_value=1;
	
	_button.state[DN_3D].port=GPIO_PORTK_BASE;
	_button.state[DN_3D].pin=GPIO_PIN_3;
	_button.state[DN_3D].value=1;
	_button.state[DN_3D].last_value=1;
	
	_button.state[LT_3D].port=GPIO_PORTK_BASE;
	_button.state[LT_3D].pin=GPIO_PIN_1;
	_button.state[LT_3D].value=1;
	_button.state[LT_3D].last_value=1;
	
	_button.state[RT_3D].port=GPIO_PORTK_BASE;
	_button.state[RT_3D].pin=GPIO_PIN_0;
	_button.state[RT_3D].value=1;
	_button.state[RT_3D].last_value=1;
	
	_button.state[ME_3D].port=GPIO_PORTK_BASE;
	_button.state[ME_3D].pin=GPIO_PIN_2;
	_button.state[ME_3D].value=1;
	_button.state[ME_3D].last_value=1;
	/******************************************************************************************/
}

/***************************************
函数名:	void Read_Button_State_One(button_state *button)
说明: 按键读取状态机
入口:	button_state *button-监测按键结构体指针
出口:	无
备注:	无
作者:	无名创新
***************************************/
void Read_Button_State_One(button_state *button)
{
	button->value=GPIOPinRead(button->port,button->pin);
	if(button->value==0)
	{
		if(button->last_value!=0)//首次按下
		{
			button->press_time=millis();//记录按下的时间点
			button->in_time=millis();//记录按下的时间点
			button->in_press_cnt=0;
		}
		else
		{
			if(millis()-button->press_time>KEEP_LONG_PRESS_LIMIT)//达到持续长按时间限制，声音提示可以松开了
			{
//				beep.period=20;//20*5ms
//				beep.light_on_percent=0.5f;			
//				beep.reset=1;
//				beep.times=1;			
			}
			else if(millis()-button->in_time>IN_PRESS_LIMIT)//持续按下
			{
				button->in_time=millis();
				button->press=IN_PRESS;
				if(button->press==IN_PRESS)  button->in_press_cnt++;
			}
			
		}
	}
  else
	{
		if(button->last_value==0)//按下后释放
		{
			button->release_time=millis();//记录释放时的时间
			if(button->release_time-button->press_time>KEEP_LONG_PRESS_LIMIT)//持续长按按键5S
			{
				button->press=KEEP_LONG_PRESS;
				button->state_lock_time=0;
				
				beep.period=200;//200*5ms
				beep.light_on_percent=0.5f;			
				beep.reset=1;
				beep.times=3;				
			}
			else if(button->release_time-button->press_time>LONG_PRESS_LIMIT)//长按按键1S
			{
			  button->press=LONG_PRESS;
				button->state_lock_time=0;//5ms*300=1.5S
				
				beep.period=200;//200*5ms
				beep.light_on_percent=0.5f;			
				beep.reset=1;
				beep.times=1;
			}
			else
			{
			  button->press=SHORT_PRESS;
				button->state_lock_time=0;//5ms*300=1.5S
				
				beep.period=20;//200*5ms
				beep.light_on_percent=0.5f;			
				beep.reset=1;
				beep.times=1;
			}
		}
	}
	button->last_value=button->value;
	
	if(button->press==LONG_PRESS
	 ||button->press==SHORT_PRESS)//按下释放后，程序后台1.5S内无响应，复位按键状态
	{
	  button->state_lock_time++;
		if(button->state_lock_time>=300)
		{			
			 button->press=NO_PRESS;
			 button->state_lock_time=0;
		}
	}
}

/***************************************
函数名:	void read_button_state_all(void)
说明: 读取所有按键状态
入口:	无
出口:	无
备注:	无
作者:	无名创新
***************************************/
void read_button_state_all(void)
{
  for(uint16_t i=0;i<BUTTON_NUM;i++)
	{
	  Read_Button_State_One(&_button.state[i]);
	}
}

