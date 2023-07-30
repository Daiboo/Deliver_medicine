#include "Headfile.h"
#include "tm4c12x_it.h"



/***************************************
函数名:	void UART0_IRQHandler(void)
说明: UART0中断服务函数
入口:	无
出口:	无
备注:	无
作者:	无名创新
***************************************/
void UART0_IRQHandler(void)//UART0中断函数
{	
  uint32_t flag = UARTIntStatus(UART0_BASE,1);//获取中断标志 原始中断状态 不屏蔽中断标志		
  UARTIntClear(UART0_BASE,flag);	 //清除中断标志		
  while(UARTCharsAvail(UART0_BASE))	 //判断FIFO是否还有数据			
  {
		uint8_t ch=UARTCharGet(UART0_BASE);
		UARTCharPut(UART0_BASE,ch);
  }
}


/***************************************
函数名:	void UART1_IRQHandler(void)
说明: UART1中断服务函数
入口:	无
出口:	无
备注:	无
作者:	无名创新
***************************************/
void UART1_IRQHandler(void)//UART1中断函数
{				
  uint32_t flag = UARTIntStatus(UART1_BASE,1);//获取中断标志 原始中断状态 屏蔽中断标志	
  UARTIntClear(UART1_BASE,flag);//清除中断标志	
  while(UARTCharsAvail(UART1_BASE))//判断FIFO是否还有数据		
  {			
		uint8_t ch=UARTCharGet(UART1_BASE);
		UARTCharPut(UART1_BASE,ch);
  }
}

/***************************************
函数名:	void UART2_IRQHandler(void)
说明: UART2中断服务函数
入口:	无
出口:	无
备注:	无
作者:	无名创新
***************************************/
void UART2_IRQHandler(void)
{
  uint32_t flag = UARTIntStatus(UART2_BASE,1);//获取中断标志 原始中断状态 屏蔽中断标志		
  UARTIntClear(UART2_BASE,flag);//清除中断标志	
  while(UARTCharsAvail(UART2_BASE))//判断FIFO是否还有数据				
  {		
		uint8_t ch=UARTCharGet(UART2_BASE);
		UARTCharPut(UART2_BASE,ch);
  }
}


/***************************************
函数名:	void UART3_IRQHandler(void)
说明: UART3中断服务函数--无名创新地面站通讯串口(UART3):
入口:	无
出口:	无
备注:	无
作者:	无名创新
***************************************/
void UART3_IRQHandler(void)
{		
 uint32_t flag = UARTIntStatus(UART3_BASE,1);//获取中断标志 原始中断状态 屏蔽中断标志		
 UARTIntClear(UART3_BASE,flag);//清除中断标志			
 while(UARTCharsAvail(UART3_BASE))//判断FIFO是否还有数据		
 {	
		uint8_t ch=UARTCharGet(UART3_BASE);

		Raspi_Data_Phrase_Prepare_Lite(ch);   // 树莓派发送指令解析函数
    // raspi_ctrl_procedure.left_pwm = ch;
	}	
}

/***************************************
函数名:	void UART4_IRQHandler(void)
说明: UART4中断服务函数
入口:	无
出口:	无
备注:	无
作者:	无名创新
***************************************/
//void UART4_IRQHandler(void)//UART4中断函数
//{				
//  uint32_t flag = UARTIntStatus(UART4_BASE,1);//获取中断标志 原始中断状态 屏蔽中断标志		
//  UARTIntClear(UART4_BASE,flag);//清除中断标志	
//  while(UARTCharsAvail(UART4_BASE))//判断FIFO是否还有数据		
//  {
//		uint8_t ch=UARTCharGet(UART4_BASE);	
//		bluetooth_app_prase(ch);
//  }
//}

/***************************************
函数名:	void UART5_IRQHandler(void)
说明: UART5中断服务函数--超声波测距传感器接口(UART5)
入口:	无
出口:	无
备注:	无
作者:	无名创新
***************************************/
void UART5_IRQHandler(void)//UART5中断函数
{				
 uint32_t flag = UARTIntStatus(UART5_BASE,1);//获取中断标志 原始中断状态 屏蔽中断标志		
 UARTIntClear(UART5_BASE,flag);//清除中断标志	
 while(UARTCharsAvail(UART5_BASE))//判断FIFO是否还有数据		
 {
		uint8_t ch=UARTCharGet(UART5_BASE);
		if(com5_rx_cnt>=2) com5_rx_cnt=0;
		com5_rx_buf[com5_rx_cnt++]=ch;
 }
}

/***************************************
函数名:	void UART6_IRQHandler(void)
说明: UART6中断服务函数
入口:	无
出口:	无
备注:	无
作者:	无名创新
***************************************/
//void UART6_IRQHandler(void)
//{		
//  uint32_t flag = UARTIntStatus(UART6_BASE,1);//获取中断标志 原始中断状态 屏蔽中断标志		
//  UARTIntClear(UART6_BASE,flag);//清除中断标志	
//  while(UARTCharsAvail(UART6_BASE))//判断FIFO是否还有数据		
//  {
//		uint8_t ch=UARTCharGet(UART6_BASE);
//		UARTCharPut(UART6_BASE,ch);
//  }
//}

/***************************************
函数名:	void UART7_IRQHandler(void)
说明: UART7中断服务函数--OPENMV视觉通讯串口(UART7)
入口:	无
出口:	无
备注:	无
作者:	无名创新
***************************************/
void UART7_IRQHandler(void)//UART7中断函数
{		
  uint32_t flag = UARTIntStatus(UART7_BASE,1);//获取中断标志 原始中断状态 屏蔽中断标志	
  UARTIntClear(UART7_BASE,flag);//清除中断标志		
  while(UARTCharsAvail(UART7_BASE))//判断FIFO是否还有数据			
  {			
		uint8_t ch=UARTCharGet(UART7_BASE);
		SDK_Data_Receive_Prepare_1(ch);
  }
}


