#ifndef __HEADFILE_H
#define __HEADFILE_H

#if 0
#define keyup_port  GPIO_PORTJ_BASE
#define keyup_pin   GPIO_PIN_1
#define keydown_port  GPIO_PORTJ_BASE
#define keydown_pin   GPIO_PIN_0

#define heater_port GPIO_PORTC_BASE
#define heater_pin  GPIO_PIN_2
#else

#define keydown_port  GPIO_PORTJ_BASE
#define keydown_pin   GPIO_PIN_0
#define keyup_port    GPIO_PORTC_BASE
#define keyup_pin     GPIO_PIN_2

// **************加热功能****************
#define heater_port GPIO_PORTJ_BASE
#define heater_pin  GPIO_PIN_1

#endif

#define KEYUP  GPIOPinRead(keyup_port  ,keyup_pin)
#define KEYDN  GPIOPinRead(keydown_port,keydown_pin)




// *******************中断优先级**********************
#define  USER_INT0  0x00   //PPM     遥控器PPM数据解析  0x00<<6
#define  USER_INT1  0x20   //UART6   GPS数据解析/ROS通讯串口	921600/460800
#define  USER_INT2  0x40   //UART0   底部OPENMV数据解析	256000
                           //UART3   前部OPENMV数据解析	256000
													 
#define  USER_INT3  0x60   //UART7   激光测距通讯串口   921600
													 //UART1   无名创新地面站串口	921600
													 
#define  USER_INT4  0x80   //UART2   光流数据解析19200/激光雷达230400


#define  USER_INT5  0xA0	 //TIMER1  1ms
#define  USER_INT6  0xC0   //TIMER0		 5ms
#define  USER_INT7  0xE0   //TIMER2    10ms




// *******************系统头文件*********************
#include "stdio.h"
#include <stdint.h>
#include <stdbool.h>
#include "hw_memmap.h"
#include "hw_types.h"
#include "hw_gpio.h"
#include "hw_ints.h"
#include "pin_map.h"
#include "fpu.h"
#include "interrupt.h"
#include "Time.h"
#include "sysctl.h"
#include "systick.h"
#include "eeprom.h"
#include "gpio.h"
#include "timer.h"
#include "pwm.h"
#include "qei.h"
#include "uart.h"
#include "uartstdio.h"

// *************************用户自定义头文件******************************

#include "system.h"
#include "neeprom.h"
#include "Datatype.h"
#include "ssd1306.h"
#include "oled.h"
#include "rgb.h"
#include "nadc.h"
#include "Buzzer.h"
#include "NTimer.h"
#include "ni2c.h"
#include "npwm.h"
#include "nuart.h"
#include "Ultrasonic.h"

#include "wp_math.h"
#include "filter.h"
#include "nEncoder.h"
#include "icm20608.h"
#include "Sensor.h"

#include "motor_control.h"
#include "pid.h"

#include "sdk.h"
#include "nbutton.h"
#include "vision.h"
#include "develop_mode.h"
#include "gray_detection.h"
#include "FusionAhrs.h"
#include "FusionOffset.h"
#include "subtask.h"
#include "ui.h"
#include "raspi_undercontrol.h"
#endif
