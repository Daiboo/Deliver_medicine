#include "Headfile.h"
#include "vision.h"




unsigned char sdk_data_to_send[10]; 

/**
 * @brief 向openmv发送任务指定命令
*/
void Tidata_Tosend_OpenMV(uint8_t mode)
{
	sdk_data_to_send[0]=0xFF;
	sdk_data_to_send[1]=0xFE;
	sdk_data_to_send[2]=0xA0;
	sdk_data_to_send[3]=2;
	sdk_data_to_send[4]=mode;
	sdk_data_to_send[5]=0;
	
	uint8_t sum = 0;
	for(uint8_t i=0;i<6;i++) sum += sdk_data_to_send[i];
	sdk_data_to_send[6]=sum;
	UART_SendBytes(7,sdk_data_to_send, 7);
}




#define SDK_TARGET_X_OFFSET  0
#define SDK_TARGET_Y_OFFSET  0


Target_Check camera1;    // openmv发来的数据接收
float SDK_Target_Yaw_Gyro=0;

/*************************************************************************/
#define  Pixel_Size_MV    0.0024f//6um=0.000006m=0.0006cm
                       //320---0.0012
                       //160---0.0024
                       //80 ---0.0048
#define  Focal_Length_MV  0.42f

#define OV7725_Sensor_Width_MM    		3.984f//3984um
#define OV7725_Sensor_Height_MM   		2.952f//2952um
#define Pixel_Image_Width_MV    		160//320
#define Pixel_Image_Height_MV   		120//240
#define Pixel_Image_Focal_MM_MV 		4.2f
#define Pixel_Image_View_Angle_X_MV  (56.72/2)//deg(50.75/2)
#define Pixel_Image_View_Angle_Y_MV  (44.07/2)//deg(38.72/2)
/*************************************************************************/
#define  Pixel_Size_CV    0.00056f//cm
													//2592:1944——0.00014cm
													//640:480——0.00056cm
#define  Focal_Length_CV  0.36f  //焦距3.6mm

#define OV5647_Sensor_Width_MM    		3.674f//3674um
#define OV5647_Sensor_Height_MM   		2.738f//2738.4um#
#define Pixel_Image_Width_CV    			640//640
#define Pixel_Image_Height_CV   			480//480
#define Pixel_Image_Focal_MM_CV 			3.6f
#define Pixel_Image_View_Angle_X_CV  (53.5/2)//约为66deg广角
#define Pixel_Image_View_Angle_Y_CV  (41.4/2)


#define AprilTag_Side_Length  13.6f//cm13.6

float _Pixel_Image_View_Angle_X,_Pixel_Image_View_Angle_Y;
/***************************************
函数名:	void Get_Camera_Wide_Angle(float view_angle)
说明: 通过摄像头视场角计算x,y方向视角
入口:	float view_angle-视场角fov
出口:	无
备注:	无
作者:	无名创新
***************************************/
void Get_Camera_Wide_Angle(float view_angle)
{
	float fh=5.0f/tanf(0.5f*view_angle*DEG2RAD);
  _Pixel_Image_View_Angle_X=2*RAD2DEG*atanf(4/fh);
  _Pixel_Image_View_Angle_Y=2*RAD2DEG*atanf(3/fh);
}

uint16_t _CX=0,_CY=0;
float _P1=0,_P2=0,_TX=0,_TY=0,_DX=0,_DY=0;
/***************************************
函数名:	void Sensor_Parameter_Sort( uint16_t tx,uint16_t ty,float pitch,float roll,float alt)
说明: 从图像坐标到实际距离偏移的转换
入口:	uint16_t tx-图像坐标x
			uint16_t ty-图像坐标y
			float pitch-俯仰角
			float roll-横滚角
			float alt-距离
出口:	无
备注:	无
作者:	无名创新
***************************************/
void Sensor_Parameter_Sort(uint16_t tx,uint16_t ty,float pitch,float roll,float alt)
{
//	Get_Camera_Wide_Angle(68);//根据摄像头广角，计算得到X、Y轴视角
	float theta_x_max=0,theta_y_max=0;
	theta_x_max=Pixel_Image_View_Angle_X_MV;
	theta_y_max=Pixel_Image_View_Angle_Y_MV;
  _P1=0.5f*Pixel_Image_Width_MV/tanf(theta_x_max*DEG2RAD);	
	_P2=0.5f*Pixel_Image_Height_MV/tanf(theta_y_max*DEG2RAD);
	
	_CX=Pixel_Image_Width_MV/2;
	_CY=Pixel_Image_Height_MV/2;
	
	float tmp_x=0,tmp_y=0;
	tmp_x=atanf((_CX-tx)/_P1);
	tmp_y=atanf((_CY-ty)/_P2);
	
	_TX= tanf(tmp_x+roll*DEG2RAD) *_P1;
	_TY= tanf(tmp_y+pitch*DEG2RAD)*_P2;
	
//	_DX=alt*_TX/_P1;
//	_DY=alt*_TY/_P2;
	_DX=0.5f*alt*_TX/_P1;
	_DY=0.5f*alt*_TY/_P2;

	camera1.sdk_target.x=_DX;
  camera1.sdk_target.y=_DY;
}


/***************************************
函数名:	void SDK_DT_Reset(void)
说明: SDK数据结构体复位
入口:	无
出口:	无
备注:	无
作者:	无名创新
***************************************/
void SDK_DT_Reset(void)
{
	camera1.x=0;
	camera1.flag=0;
	camera1.fps=0;

	
}

/***************************************
函数名:	void Openmv_Data_Receive_Anl_1(uint8_t *data_buf,uint8_t num,Target_Check *target)
说明: openmv1数据解析函数
入口:	uint8_t *data_buf-待解析数据
			uint8_t num-待解析数据长度
			Target_Check *target-目标检测结构体
出口:	无
备注:	无
作者:	无名创新
***************************************/
void Openmv_Data_Receive_Anl_1(uint8_t *data_buf,uint8_t num,Target_Check *target)
{
  uint8_t sum = 0;
  for(uint8_t i=0;i<(num-1);i++)  sum+=*(data_buf+i);
  if(!(sum==*(data_buf+num-1))) 	return; //不满足和校验条件
  if(!(*(data_buf)==0xFF && *(data_buf+1)==0xFC))return; //不满足帧头条件

	target->task=*(data_buf+2);  // 获取任务类别

	switch(target->task)
	{

		case Tracking_task:  // 视觉任务
		{
			
			target->x = *(data_buf+4);  // 发来的是前8位
			target->rho = *(data_buf+5);
			
			target->cross = *(data_buf+6);
			target->flag = *(data_buf+7);
			target->fps = *(data_buf+8);


			target->target_ctrl_enable=target->flag;   // 检测到了为1，一般情况下都为1
			if(target->flag!=0)  
			{
				if(target->trust_cnt_tracking_or_cross<20)	 
				{
					target->trust_cnt_tracking_or_cross++;
					target->trust_flag_tracking_or_cross = 0;
				}
				else target->trust_flag_tracking_or_cross = 1;
			}
			else 
			{
				target->trust_cnt_tracking_or_cross /= 2;
				target->trust_flag_tracking_or_cross = 0;
			}	
			gray_status[1] = target->x;	vision_status_worse/=2;
			rho_status[1] = target->rho;

		}
		case Number_recognition_inbegin_task:
		{

			target->inbegin_recognition_finsh_flag = *(data_buf+4);
			
		}
		break;
		case Number_recognition_intrack_task:  
		{
			target->intrack_todo_task = *(data_buf+4);


			if(target->intrack_todo_task != 7)  
			{
				if(target->trust_cnt_number_recongition < 10)	 
				{
					target->trust_cnt_number_recongition++;
					target->trust_flag_number_recongition = 0;
				}
				else target->trust_flag_number_recongition = 1;
			}
			else 
			{
				target->trust_cnt_number_recongition /= 2;
				target->trust_flag_number_recongition = 0;
			}

			
		}
		break;
		default:  
		{
			target->target_ctrl_enable = 0;
			
			target->x=0;
		}
	}

}



static uint8_t state[2] = {0};
static uint8_t _data_len[2] = {0},_data_cnt[2] = {0};
static uint8_t _buf[2][SDK_Target_Length];
/***************************************************
函数名: void SDK_Data_Receive_Prepare_1(uint8_t data)
说明:	openmv1数据解析状态机
入口:	uint8_t data-当前待解析字节
出口:	无
备注:	无
作者:	无名创新
****************************************************/
void SDK_Data_Receive_Prepare_1(uint8_t data)
{
	uint8_t label=0;
  if(state[label]==0&&data==0xFF)//帧头1
  {
    state[label]=1;
    _buf[label][0]=data;
  }
  else if(state[label]==1&&data==0xFC)//帧头2
  {
    state[label]=2;
    _buf[label][1]=data;
  }
  else if(state[label]==2&&data<0XFF)//任务字节
  {
    state[label]=3;
    _buf[label][2]=data;
  }
  else if(state[label]==3&&data<50)//数据长度
  {
    state[label] = 4;
    _buf[label][3]=data;
    _data_len[label] = data;
    _data_cnt[label] = 0;
  }
  else if(state[label]==4&&_data_len[label]>0)//有多少数据长度，就存多少个
  {
    _data_len[label]--;
    _buf[label][4+_data_cnt[label]++]=data;
    if(_data_len[label]==0) state[label] = 5;
  }
  else if(state[label]==5)//最后接收数据校验和
  {
    state[label] = 0;
    _buf[label][4+_data_cnt[label]]=data;
		Openmv_Data_Receive_Anl_1(_buf[label],_data_cnt[label]+5,&camera1);  // 数据解析
  }
  else state[label] = 0;
}



