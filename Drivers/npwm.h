#ifndef __NPWM_H
#define __NPWM_H
#include "Datatype.h"

#define PWM_GEN_MODE PWM_GEN_MODE_UP_DOWN   // Up/Down count mode
#define PWM_GEN_SYNC_MODE PWM_GEN_MODE_NO_SYNC   // Immediate updates
#define MOTOR_PWM_SYSCTL_PWMDIV   SYSCTL_PWMDIV_64   //80M/64=1.25M=0.8us   PWM clock is processor clock /64
#define MOTOR_PERIOD_MAX_800US    1000    //电机PWM频率——1.25khz
#define MOTOR_PERIOD_MAX_2500US   3125
#define MOTOR_PERIOD_MAX_5000US   6250
#define MOTOR_PERIOD_MAX_10000US  12500
#define MOTOR_PERIOD_MAX_20000US  25000


void PWM0_Init(void);
void PWM1_Init(void);

void PWM_Output(uint16_t width1,uint16_t width2,uint16_t width3,uint16_t width4);

void steer_servo_pwm_m1p0(uint16_t us);
void steer_servo_pwm_m1p1(uint16_t us);
void steer_servo_pwm_m1p2(uint16_t us);
void steer_servo_pwm_m1p3(uint16_t us);


#endif
