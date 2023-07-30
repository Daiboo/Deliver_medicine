#ifndef __DATATYPE_H
#define __DATATYPE_H

#include <stdint.h>
#include <math.h>

typedef enum
{
	LOCK 		=0x00,
	UNLOCK  =0x01,
}LOCK_STATE;


#define ABS(X)  (((X)>0)?(X):-(X))
#define MAX(a,b)  ((a)>(b)?(a):(b))
#define MIN(a,b)  ((a)>(b)?(b):(a))

// **********加速度传感器陀螺仪**********
enum 
{
	_ROL=0,
	_PIT,
	_YAW
};

enum 
{
	ROL=0,
	PIT,
	YAW
};

typedef struct
{
  float x;
  float y;
  float z;
}vector3f;

typedef struct
{
  float x;
  float y;
}vector2f;


typedef enum
{
	WHO_AM_I_MPU6050  =0x68,
	WHO_AM_I_ICM20689 =0x98,
	WHO_AM_I_ICM20608D=0xAE,
	WHO_AM_I_ICM20608G=0xAF,
	WHO_AM_I_ICM20602=0x12,
}IMU_ID_READ;



// **********低电压报警**********
typedef struct
{
	float value;
	uint8_t enable;
	float upper;
	float lower;
	uint16_t low_vbat_cnt;
}low_voltage;


// ********记录时间*************
typedef struct
{
  volatile float last_time;
  volatile float current_time;
  volatile float period;
  volatile uint16_t period_int;//单位ms
}systime;


// *********************两轮子小车参数***************
typedef struct
{
	float speed;   // 速度
	float azimuth; // 方位角
	float w;		
	vector2f pos;
	vector2f vel;
	float distance;
}two_wheel_model;

// *********************电机控制*********************
typedef struct
{
	int8_t left_encoder_dir_config,right_encoder_dir_config;//编码器方向配置
	int8_t left_motion_dir_config	,right_motion_dir_config; //电机运动方向配置
	float wheel_radius_cm;				//轮胎半径,单位为cm
	uint16_t pulse_num_per_circle;//轮胎转动一圈累计的脉冲数量
}motor_config;

// ************************编码器********************
typedef struct
{
	int32_t left_motor_cnt,right_motor_cnt;//单个采样周期内的脉冲数量
	float left_motor_dir,right_motor_dir; //运动方向
	float left_motor_speed_rpm,right_motor_speed_rpm;//转速单位转每分钟
	float left_motor_gyro_rps,right_motor_gyro_rps;//转速单位rad/s
	float left_motor_speed_cmps,right_motor_speed_cmps;//转速c单位为cm/s
	float left_motor_period_ms,right_motor_period_ms;
	
	int32_t left_motor_total_cnt,right_motor_total_cnt;
	int32_t left_motor_period_cnt,right_motor_period_cnt;
	uint8_t left_motor_cnt_clear,right_motor_cnt_clear;
	
}encoder;

// **************************控制器**************************






// *************************灰度**************************
typedef struct
{
	uint8_t bit1	:1;
	uint8_t bit2	:1;
	uint8_t bit3	:1;
	uint8_t bit4	:1;
	uint8_t bit5	:1;
	uint8_t bit6	:1;
	uint8_t bit7	:1;
	uint8_t bit8	:1;
	uint8_t bit9	:1;
	uint8_t bit10	:1;
	uint8_t bit11	:1;
	uint8_t bit12	:1;
	uint8_t bit13	:1;
	uint8_t bit14	:1;
	uint8_t bit15	:1;
	uint8_t bit16	:1;
}gray_flags;





// *********************传感器汇总************************
typedef struct
{
	vector3f _gyro_dps_raw,gyro_dps_raw;   // 原始陀螺仪数据
	vector3f _accel_g_raw,accel_g_raw;		// 原始加速度数据
	vector3f mag_tesla_raw;
	vector3f last_mag_raw;
	float temperature_raw,last_temperature_raw;
	float temperature_filter;             // 得到计算后的温度值
	float vbat;
	//校准后的数据
	vector3f gyro_dps;
	vector3f accel_g;
	vector3f mag_tesla;
	
	//
	vector3f gyro_offset;
	vector3f accel_scale,accel_offset;
	
	//
	float left_motor_speed_cmps;
	float right_motor_speed_cmps;
	float average_speed_cmps;
	
	
	uint8_t quaternion_init_ok;
	float quaternion_init[4];//初始四元数
	float quaternion[4];//四元数
	float rpy_deg[3];      // 姿态角
	float rpy_gyro_dps[3];
	float rpy_gyro_dps_enu[3];
	vector3f accel_earth_cmpss;
	vector2f accel_body_cmpss;
	float sin_rpy[3];
	float cos_rpy[3];
	float cb2n[9];
	float rpy_obs_deg[3];//观测姿态角度
	float rpy_kalman_deg[3];
	//
	float yaw_gyro_enu;  // 
	//
	uint16_t imu_convergence_cnt;
	uint8_t imu_convergence_flag;
	uint8_t temperature_stable_flag;
	uint8_t imu_cal_flag;
	uint8_t imu_health;
	uint8_t lpf_init;
	two_wheel_model state_estimation;
}sensor;



#endif
