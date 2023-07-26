#ifndef __SYSTEM_H
#define __SYSTEM_H

#include "Datatype.h"


void System_Init(void);

void delay(uint32_t ms);
void delayMicroseconds(uint32_t us);
uint32_t millis(void);
uint32_t micros(void);
void Delay_Ms(uint32_t x);
void Delay_Us(uint32_t x);
void delay_ms(uint32_t x);
void delay_us(uint32_t x);

void get_systime(systime *sys);


#endif


