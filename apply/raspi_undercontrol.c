#include "Headfile.h"
#include "raspi_undercontrol.h"




static uint8_t Raspi_Head[2]={0xFF,0xFC};//数据帧头
static uint8_t Raspi_End[2] ={0xA1,0xA2};//数据帧尾

uint8_t tidata_tosend_raspi[10];  // 要发送给树莓派的数据




/**
 * @brief ti板子要发送给树莓派的数据
*/
void Tidata_Tosend_Raspi(unsigned char mode,COM_SDK com)
{
    tidata_tosend_raspi[0]=0xFF;
    tidata_tosend_raspi[1]=0xFE;
    tidata_tosend_raspi[2]=0xA0; // 功能字
    tidata_tosend_raspi[3]=2;   // 长度
    tidata_tosend_raspi[4]=mode; // 功能
    tidata_tosend_raspi[5]=0;  // 和校验
    uint8_t sum = 0;
    for(uint8_t i=0;i<6;i++) sum += tidata_tosend_raspi[i];
    tidata_tosend_raspi[6]=sum;
    UART_SendBytes(3,tidata_tosend_raspi, 7);

}





uint32_t Raspi_receive_fault_cnt=0;
static uint8_t Raspi_Receivebuf[100];

/**
 * @brief 解析来自树莓派发送的数据
 * @param data 数据
*/
void Raspi_Data_Phrase_Prepare_Lite(uint8_t data)
{
    static uint8_t data_len = 0, data_cnt = 0;
    static uint8_t state = 0;

    if(state==0 && data==Raspi_Head[1])//判断帧头1 0xFC
    {
        state=1;
        Raspi_Receivebuf[0]=data;
    }
    else if(state==1 && data==Raspi_Head[0])//判断帧头2 0xFF
    {
        state=2;
        Raspi_Receivebuf[1]=data;
    }
    else if(state==2 && data<0XF1)  //功能字节，树莓派发送命令控制
    {
        state=2;
        Raspi_Receivebuf[2]=data;
    }
    else if(state==3&&data<100)//有效数据长度
    {
        state = 4;
        Raspi_Receivebuf[3]=data;
        data_len = data;   // 有效数据长度
        data_cnt = 0;
    }
    else if(state==4 && data_len>0) //数据接收 
    {
        data_len--;
        Raspi_Receivebuf[4+data_cnt++]=data;
        if(data_len==0)  state = 5;
    }
    else if(state==5)//异或校验位
    {
        state = 6;
        Raspi_Receivebuf[4+data_cnt++]=data;
    }
    
	else if(state==6 && data==Raspi_End[0])//帧尾0   0xA1
	{
			state = 7;
			Raspi_Receivebuf[4+data_cnt++]=data;
	}
	else if(state==7&&data==Raspi_End[1])//帧尾1 0xA2
	{
			state = 0;
			Raspi_Receivebuf[4+data_cnt]=data;
		    Raspi_Data_Phrase_Process_Lite(Raspi_Receivebuf,data_cnt+5);//数据解析
	}
    else 
	{
		state = 0;
		Raspi_receive_fault_cnt++;
	}

}

void Raspi_Data_Phrase_Process_Lite(uint8_t *data_buf,uint8_t num)  //树莓派数据解析进程
{
    uint8_t sum = 0;
    for(uint8_t i=0;i<(num-3);i++)  sum ^= *(data_buf+i);    // 不包括最后三位
    if(!(sum==*(data_buf+num-3)))
        if(!(*(data_buf)==Raspi_Head[1]&&*(data_buf+1)==Raspi_Head[0]))         return;//判断帧头
        if(!(*(data_buf+num-2)==Raspi_End[0]&&*(data_buf+num-1)==Raspi_End[1])) return;//帧尾校验 
    
    // if(*(data_buf+2) == 0x09)  // 第三位为功能字节
    // {
        
    // }
    // else if(*(data_buf+2) == 0x0F)
}   