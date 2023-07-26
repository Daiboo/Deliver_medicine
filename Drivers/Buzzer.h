#ifndef __NGPIO_H
#define __NGPIO_H

#include <stdint.h>


typedef struct
{
	uint16_t times;				 //预设闪烁总次数
	uint8_t  reset;				 //闪烁进程复位标志
	uint16_t cnt;					 //闪烁控制计数器
	uint16_t times_cnt;		 //记录已闪烁次数
	uint8_t  end;					 //闪烁完成标志位
	uint32_t port;				 //闪烁所在的端口
	uint8_t pin;					 //闪烁所在的GPIO
	uint32_t period;			 //闪烁周期
	float light_on_percent;//单个周期内点亮时间百分比   占空比
}_laser_light;


extern _laser_light  beep;
void Buzzer_Init(void);
void laser_light_work(_laser_light *light);

#endif

