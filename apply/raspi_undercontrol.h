#ifndef __RASPI_UNDERCONTROL_H
#define __RASPI_UNDERCONTROL_H
#include <stdint.h>



void Raspi_Data_Phrase_Prepare_Lite(uint8_t data);
void Raspi_Data_Phrase_Process_Lite(uint8_t *data_buf,uint8_t num);  //树莓派数据解析进程
#endif
