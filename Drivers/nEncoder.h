#ifndef __NENCODER_H
#define __NENCODER_H
#include "Headfile.h"

#define quadrature_decoder_enable 1//正交解码使能


void Encoder_Init(void);
float get_left_motor_speed(void);
float get_right_motor_speed(void);

extern encoder NEncoder;

#endif




