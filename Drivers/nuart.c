#include "Headfile.h"
#include "tm4c12x_it.h"
#include "nuart.h"

/***************************************
函数名:	void UART_SendBytes(uint32_t port,uint8_t *ubuf, uint32_t len)
说明: 发送N个字节长度的数据
入口:	uint32_t port-串口号
		uint8_t *ubuf-待发生数据地址 
		uint32_t len-待发送数据长度
出口:	无
备注:	无
作者:	无名创新
***************************************/
void UART_SendBytes(uint32_t port,uint8_t *ubuf, uint32_t len)
{
	uint32_t ui32Base=UART0_BASE;
	switch(port)
	{
		case 0:		ui32Base=UART0_BASE;	break;
		case 1:		ui32Base=UART1_BASE;	break;
		case 2:		ui32Base=UART2_BASE;	break;
		case 3:		ui32Base=UART3_BASE;	break;
		case 4:		ui32Base=UART4_BASE;	break;
		case 5:		ui32Base=UART5_BASE;	break;
		case 6:		ui32Base=UART6_BASE;	break;
		case 7:		ui32Base=UART7_BASE;	break;
		default:  ui32Base=UART0_BASE;
	}
	while(len--)
	{
		UARTCharPut(ui32Base, *ubuf++);
	}
}

/***************************************
函数名:	void UART_SendByte(uint32_t port,uint8_t data)
说明: 发送1个字节长度的数据
入口:	uint32_t port-串口号
			uint8_t data-待发生数据
出口:	无
备注:	无
作者:	无名创新
***************************************/
void UART_SendByte(uint32_t port,uint8_t data)
{
	uint32_t ui32Base=UART0_BASE;
	switch(port)
	{
		case 0:		ui32Base=UART0_BASE;	break;
		case 1:		ui32Base=UART1_BASE;	break;
		case 2:		ui32Base=UART2_BASE;	break;
		case 3:		ui32Base=UART3_BASE;	break;
		case 4:		ui32Base=UART4_BASE;	break;
		case 5:		ui32Base=UART5_BASE;	break;
		case 6:		ui32Base=UART6_BASE;	break;
		case 7:		ui32Base=UART7_BASE;	break;
		default:  ui32Base=UART0_BASE;
	}
   UARTCharPut(ui32Base,data);
}

/***************************************
函数名:	void UART0_Init(unsigned long bound)
说明: 串口0资源初始化--山外地面站通讯串口(UART0):
入口:	unsigned long bound-波特率
出口:	无
备注:	无
作者:	无名创新
***************************************/
//void UART0_Init(unsigned long bound)//串口0初始化
//{
//  SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);//使能GPIO外设		
//  SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);//使能UART外设
//  GPIOPinConfigure(GPIO_PA0_U0RX);//GPIO模式配置 PA0--RX PA1--TX 
//  GPIOPinConfigure(GPIO_PA1_U0TX);
//  GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);//GPIO的UART模式配置
//	//UARTConfigSetExpClk(UART0_BASE, SysCtlClockGet(), bound,
//	//									(UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE |
//	//									 UART_CONFIG_PAR_NONE));
//	UARTClockSourceSet(UART0_BASE, UART_CLOCK_PIOSC);
//	UARTStdioConfig(0, bound, 16000000);
//	
//  //UART协议配置 波特率115200 8位 1停止位  无校验位	
//  //UART禁用FIFO 默认FIFO Level为4/8 寄存器满8字节后产生中断	//禁用后接收1位就产生中断	
//  UARTFIFODisable(UART0_BASE);//使能UART0中断	IntEnable(INT_UART0);	
//  UARTIntEnable(UART0_BASE,UART_INT_RX);//使能UART0接收中断		
//  UARTIntRegister(UART0_BASE,UART0_IRQHandler);//UART中断地址注册	
//  IntPrioritySet(INT_UART0, USER_INT2);
//	
//}

/***************************************
函数名:	void UART1_Init(unsigned long bound)
说明: 串口1资源初始化--预留
入口:	unsigned long bound-波特率
出口:	无
备注:	无
作者:	无名创新
***************************************/
//void UART1_Init(unsigned long bound)//串口1初始化
//{
//  SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);//使能GPIO外设		
//  SysCtlPeripheralEnable(SYSCTL_PERIPH_UART1);//使能UART外设
//  GPIOPinConfigure(GPIO_PB0_U1RX);//GPIO模式配置 PB0--RX PB1--TX 
//  GPIOPinConfigure(GPIO_PB1_U1TX);
//  GPIOPinTypeUART(GPIO_PORTB_BASE, GPIO_PIN_0 | GPIO_PIN_1);//GPIO的UART模式配置
//	UARTConfigSetExpClk(UART1_BASE, SysCtlClockGet(), bound,
//                      (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE |
//                       UART_CONFIG_PAR_NONE));
//  UARTFIFODisable(UART1_BASE);//使能UART1中断	
//  UARTIntEnable(UART1_BASE,UART_INT_RX);//使能UART1接收中断		
//  UARTIntRegister(UART1_BASE,UART1_IRQHandler);//UART1中断地址注册	
//  IntPrioritySet(INT_UART1, USER_INT1);//USER_INT1
//}


/***************************************
函数名:	void UART2_Init(unsigned long bound)
说明: 串口2资源初始化--预留
入口:	unsigned long bound-波特率
出口:	无
备注:	无
作者:	无名创新
***************************************/
void UART2_Init(unsigned long bound)//串口6初始化
{
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);//使能GPIO外设		
  SysCtlPeripheralEnable(SYSCTL_PERIPH_UART2);//使能UART外设
  
  HWREG(GPIO_PORTD_BASE + GPIO_O_LOCK) = GPIO_LOCK_KEY;//解锁PD7
  HWREG(GPIO_PORTD_BASE + GPIO_O_CR) |= GPIO_PIN_7;//确认
  HWREG(GPIO_PORTD_BASE + GPIO_O_LOCK) = 0;//重新锁定
  
  GPIOPinConfigure(GPIO_PD6_U2RX);//GPIO模式配置 PD6--RX PD7--TX 
  GPIOPinConfigure(GPIO_PD7_U2TX);
  GPIOPinTypeUART(GPIO_PORTD_BASE, GPIO_PIN_6 | GPIO_PIN_7);//GPIO的UART模式配置
  UARTConfigSetExpClk(UART2_BASE, SysCtlClockGet(), bound,
                      (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE |
                       UART_CONFIG_PAR_NONE));
  UARTFIFODisable(UART2_BASE);//使能UART2中断	
  UARTIntEnable(UART2_BASE,UART_INT_RX);//使能UART6接收中断		
	
  UARTIntRegister(UART2_BASE,UART2_IRQHandler);//UART中断地址注册	
  IntPrioritySet(INT_UART2, USER_INT4);
}


/***************************************
函数名:	void UART3_Init(unsigned long bound)
说明: 串口3资源初始化--无名创新地面站通讯串口(UART3)/树莓派端通信串口:
入口:	unsigned long bound-波特率
出口:	无
备注:	无
作者:	无名创新
***************************************/
void UART3_Init(unsigned long bound)//串口3初始化
{
 SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);//使能GPIO外设		
 SysCtlPeripheralEnable(SYSCTL_PERIPH_UART3);//使能UART外设
 GPIOPinConfigure(GPIO_PC6_U3RX);//GPIO模式配置 PC6--RX PC7--TX 
 GPIOPinConfigure(GPIO_PC7_U3TX);
 GPIOPinTypeUART(GPIO_PORTC_BASE, GPIO_PIN_6 | GPIO_PIN_7);//GPIO的UART模式配置
 UARTConfigSetExpClk(UART3_BASE, SysCtlClockGet(), bound,
                     (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE |
                      UART_CONFIG_PAR_NONE));
 UARTFIFODisable(UART3_BASE);//使能UART0中断	
 UARTIntEnable(UART3_BASE,UART_INT_RX);//使能UART3接收中断		
 UARTIntRegister(UART3_BASE,UART3_IRQHandler);//UART3中断地址注册	
 IntPrioritySet(INT_UART3, USER_INT3);
}


/***************************************
函数名:	void UART4_Init(unsigned long bound)
说明: 串口4资源初始化--蓝牙调试器APP串口(UART4/BT)
入口:	unsigned long bound-波特率
出口:	无
备注:	无
作者:	无名创新
***************************************/
//void UART4_Init(unsigned long bound)//串口4初始化
//{
//  SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);//使能GPIO外设		
//  SysCtlPeripheralEnable(SYSCTL_PERIPH_UART4);//使能UART外设
//  GPIOPinConfigure(GPIO_PC4_U4RX);//GPIO模式配置 PC4--RX PC5--TX 
//  GPIOPinConfigure(GPIO_PC5_U4TX);
//  GPIOPinTypeUART(GPIO_PORTC_BASE, GPIO_PIN_4 | GPIO_PIN_5);//GPIO的UART模式配置
//	UARTConfigSetExpClk(UART4_BASE, SysCtlClockGet(), bound,
//										 (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE |
//										  UART_CONFIG_PAR_NONE));
//	
//  UARTFIFODisable(UART4_BASE);//使能UART4中断	
//  UARTIntEnable(UART4_BASE,UART_INT_RX);//使能UART4接收中断		
//  UARTIntRegister(UART4_BASE,UART4_IRQHandler);//UART4中断地址注册	
//  IntPrioritySet(INT_UART4, USER_INT3);//USER_INT3
//} 

/***************************************
函数名:	void UART5_Init(unsigned long bound)
说明: 串口5资源初始化--超声波测距传感器接口(UART5)
入口:	unsigned long bound-波特率
出口:	无
备注:	无
作者:	无名创新
***************************************/
void UART5_Init(unsigned long bound)//串口5初始化
{
 SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);//使能GPIO外设		
 SysCtlPeripheralEnable(SYSCTL_PERIPH_UART5);//使能UART外设
 GPIOPinConfigure(GPIO_PE4_U5RX);//GPIO模式配置 PC4--RX PC5--TX 
 GPIOPinConfigure(GPIO_PE5_U5TX);
 GPIOPinTypeUART(GPIO_PORTE_BASE, GPIO_PIN_4 | GPIO_PIN_5);//GPIO的UART模式配置
	UARTConfigSetExpClk(UART5_BASE, SysCtlClockGet(), bound,
                     (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE |
                      UART_CONFIG_PAR_NONE));
 UARTFIFODisable(UART5_BASE);//使能UART5中断	
 UARTIntEnable(UART5_BASE,UART_INT_RX);//使能UART5接收中断
 UARTIntRegister(UART5_BASE,UART5_IRQHandler);//UART5中断地址注册	
 IntPrioritySet(INT_UART5, USER_INT4);//USER_INT5
} 

/***************************************
函数名:	void UART6_Init(unsigned long bound)
说明: 串口6资源初始化--预留串口6(UART6)
入口:	unsigned long bound-波特率
出口:	无
备注:	无
作者:	无名创新
***************************************/
//void UART6_Init(unsigned long bound)
//{
//  SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);//使能GPIO外设		
//  SysCtlPeripheralEnable(SYSCTL_PERIPH_UART6);//使能UART外设
//  	
//	GPIOPinConfigure(GPIO_PD4_U6RX);//GPIO模式配置 PD4--RX PD5--TX 
//  GPIOPinConfigure(GPIO_PD5_U6TX);
//  GPIOPinTypeUART(GPIO_PORTD_BASE, GPIO_PIN_4 | GPIO_PIN_5);//GPIO的UART模式配置
//  UARTConfigSetExpClk(UART6_BASE, SysCtlClockGet(), bound,
//                      (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE |
//                       UART_CONFIG_PAR_NONE));
//  UARTFIFODisable(UART6_BASE);//使能UART0中断	
//  UARTIntEnable(UART6_BASE,UART_INT_RX);//使能UART6接收中断		
//	
//  UARTIntRegister(UART6_BASE,UART6_IRQHandler);//UART6中断地址注册	
//  IntPrioritySet(INT_UART6, USER_INT3);
//}


/***************************************
函数名:	void UART7_Init(unsigned long bound)
说明: 串口7资源初始化--OPENMV视觉通讯串口(UART7)
入口:	unsigned long bound-波特率
出口:	无
备注:	无
作者:	无名创新
***************************************/
void UART7_Init(unsigned long bound)//串口7初始化
{
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);//使能GPIO外设		
	SysCtlPeripheralEnable(SYSCTL_PERIPH_UART7);//使能UART外设
	GPIOPinConfigure(GPIO_PE0_U7RX);//GPIO模式配置 PE0--RX PE1--TX 
	GPIOPinConfigure(GPIO_PE1_U7TX);
	GPIOPinTypeUART(GPIO_PORTE_BASE, GPIO_PIN_0 | GPIO_PIN_1);//GPIO的UART模式配置
	UARTConfigSetExpClk(UART7_BASE, SysCtlClockGet(), bound,
											(UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE |
											 UART_CONFIG_PAR_NONE));	
	UARTFIFODisable(UART7_BASE);//使能UART7中断	
	UARTIntEnable(UART7_BASE,UART_INT_RX);//使能UART7接收中断		
	UARTIntRegister(UART7_BASE,UART7_IRQHandler);//UART中断地址注册	
	IntPrioritySet(INT_UART7, USER_INT2);
}

/***************************************
函数名:	void vcan_sendware(unsigned char *wareaddr, int16_t waresize)
说明: 山外发送波形处理函数
入口:	unsigned char *wareaddr-待发送数据数组
		  int16_t waresize-待发送数据长度
出口:	无
备注:	无
作者:	无名创新
***************************************/
//void vcan_sendware(unsigned char *wareaddr, int16_t waresize)
//{
//	#define CMD_WARE     3
//	uint8_t cmdf[2] = {CMD_WARE, ~CMD_WARE};//帧头
//	uint8_t cmdr[2] = {~CMD_WARE, CMD_WARE};//帧尾
//	UART_SendBytes(0,cmdf, sizeof(cmdf));
//	UART_SendBytes(0,wareaddr, waresize);
//	UART_SendBytes(0,cmdr, sizeof(cmdr));
//}


/***************************************
函数名:	void vcan_send(void)
说明: 山外发送波形
入口:	无
出口:	无
备注:	无
作者:	无名创新
***************************************/
//void vcan_send(void)
//{
//	float _nbuf[8];	
//	_nbuf[0]=balance_speed_output;
//	_nbuf[1]=balance_last_speed_output;
//	_nbuf[2]=smartcar_imu.rpy_deg[0];
//	_nbuf[3]=smartcar_imu.rpy_deg[1];
//	_nbuf[4]=smartcar_imu.rpy_obs_deg[0];
//	_nbuf[5]=smartcar_imu.rpy_obs_deg[1];	
//	_nbuf[6]=smartcar_imu.rpy_kalman_deg[1];
//	_nbuf[7]=smartcar_imu.gyro_dps.x;	
//	vcan_sendware((unsigned char *)_nbuf,sizeof(_nbuf));
//}

