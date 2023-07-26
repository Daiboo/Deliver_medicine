#ifndef __NUART_H
#define __NUART_H
#include <stdint.h>

//void UART0_Init(unsigned long bound);
//void UART1_Init(unsigned long bound);
//void UART2_Init(unsigned long bound);
//void UART3_Init(unsigned long bound);
//void UART4_Init(unsigned long bound);
void UART5_Init(unsigned long bound);
//void UART6_Init(unsigned long bound);
void UART7_Init(unsigned long bound);


void UART_SendByte(uint32_t port,uint8_t data);
void UART_SendBytes(uint32_t port,uint8_t *ubuf, uint32_t len);
void vcan_send(void);

#endif

