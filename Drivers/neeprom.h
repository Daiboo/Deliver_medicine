#ifndef __NEEPROM_H
#define __NEEPROM_H


#define PARAMETER_TABLE_STARTADDR   0x0000 
#define FLIGHT_PARAMETER_TABLE_NUM  200

typedef struct
{
   float Parameter_Table[FLIGHT_PARAMETER_TABLE_NUM];
}FLIGHT_PARAMETER;


typedef enum
{
	SPEED_SETUP=0,    	// 初始速度
	WORK_MODE,	      	// 工作模式
	ACCEL_X_OFFSET,  	// 校准时x轴加速度补偿
	ACCEL_Y_OFFSET,   	//       y
	ACCEL_Z_OFFSET, 	//       z
	GYRO_X_OFFSET,   	// 校准时x轴陀螺仪补偿
	GYRO_Y_OFFSET,		// 		y
	GYRO_Z_OFFSET,		// 		z
	CTRL_TURN_KP1,   	// 灰度管循迹时，kp
	CTRL_TURN_KI1,		// 灰度管循迹时，ki
	CTRL_TURN_KD1,		// 灰度管循迹时，kd
	CTRL_TURN_SCALE,	// 灰度管循迹时转向时，两轮差速因子
	CTRL_GYRO_KP,  		// 角速度控制kp
	CTRL_GYRO_KI,		// 角速度控制ki
	CTRL_GYRO_KD,		// 角速度控制kd
	CTRL_GYRO_SCALE,	// 两轮差速因子
	CTRL_SPEED_KP,		// 速度控制kp
	CTRL_SPEED_KI,		// 速度控制ki
	CTRL_SPEED_KD,		// 速度控制kd

	CTRL_TURN_KP2,		// openmv循迹转向kp
	CTRL_TURN_KI2,		// openmv循迹转向ki
	CTRL_TURN_KD2,  	// openmv循迹转向kd
	
	// 小车硬件参数
	LEFT_ENC_DIR_CFG,   // 左编码器方向
	RIGHT_ENC_DIR_CFG,	// 右编码器方向
	LEFT_MOVE_DIR_CFG,  // 左轮移动方向
	RIGHT_MOVE_DIR_CFG,	// 右轮移动方向
	TIRE_RADIUS_CM_CFG, // 轮胎半径
	PULSE_NPC_CFG,     	// 每圈的脉冲个数
	
	RANGEFINDER_TYPE,  // 超声波测距类型

	NO_VOLTAGE_ENABLE,  // 电压检测使能
	NO_VOLTAGE_UPPER,	// 上线
	NO_VOLTAGE_LOWER,	// 下线
	
	// 预留参数
	RESERVED_SPACE_NUM = 100,
	DELIVER_MEDICINE_SPEED,
	FIX_ROTATE_POINT,

}FLIGHT_PARAMETER_TABLE;


void EEPROM_Init(void);
void ReadFlashParameterALL(FLIGHT_PARAMETER *WriteData);
void ReadFlashParameterOne(uint16_t index,float *ReadData);
void ReadFlashParameterThree(uint16_t index,float *ReadData1,float *ReadData2,float *ReadData3);


void WriteFlashParameter(uint16_t index,float WriteData,FLIGHT_PARAMETER *Table);										 								 
void WriteFlashParameter_Two(uint16_t index,float WriteData1,float WriteData2,FLIGHT_PARAMETER *Table);														 
void WriteFlashParameter_Three(uint16_t index,float WriteData1,float WriteData2,float WriteData3,FLIGHT_PARAMETER *Table);


extern FLIGHT_PARAMETER Trackless_Params;
#endif


