#include "headfile.h"
#include "pid.h"




/***************************************************
������: void pid_control_init(controller *ctrl,
															float kp,
															float ki,
															float kd,
															float err_max,
															float integral_max,
															float output_max,
															uint8_t err_limit_flag,
															uint8_t integral_separate_flag,
															float integral_separate_limit,
															uint8_t dis_error_gap_cnt)
˵��:	pid��������ʼ��
���:	controller *ctrl-����ʼ���������ṹ��
			float kp-��������
			float ki-���ֲ���
			float kd-΢�ֲ���
			float err_max-ƫ���޷�ֵ
			float integral_max-�����޷�ֵ
			float output_max-����޷�ֵ
			uint8_t err_limit_flag-ƫ���޷���־λ
			uint8_t integral_separate_flag-���ַ����־λ
			float integral_separate_limit-���ַ����޷���־λ
			uint8_t dis_error_gap_cnt-����΢��ʱ�ļ������
����:	��
��ע:	��
****************************************************/
void pid_control_init(controller *ctrl,
											float kp,
											float ki,
											float kd,
											float err_max,
											float integral_max,
											float output_max,
											uint8_t err_limit_flag,
											uint8_t integral_separate_flag,
											float integral_separate_limit,
											uint8_t dis_error_gap_cnt)
{
	ctrl->kp=kp;
	ctrl->ki=ki;
	ctrl->kd=kd;
	ctrl->error_limit_max=err_max;
	ctrl->integral_limit_max=integral_max;
	ctrl->output_limit_max=output_max;
	ctrl->error_limit_flag=err_limit_flag;
	ctrl->integral_separate_flag =integral_separate_flag;
	ctrl->integral_separate_limit=integral_separate_limit;
	
	ctrl->dis_error_gap_cnt=dis_error_gap_cnt;
	
	ctrl->init_flag=1;
}

void  pid_integrate_reset(controller *ctrl)  {ctrl->integral=0.0f;}
/***************************************************
������: float pid_control_run(controller *ctrl)
˵��:	pid����������
���:	controller *ctrl-�������ṹ��
����:	��
��ע:	��
****************************************************/
float pid_control_run(controller *ctrl)
{
  /*******ƫ�����*********************/
	for(uint16_t i=19;i>0;i--)
	{
		ctrl->error_backup[i]=ctrl->error_backup[i-1];
	}
	ctrl->error_backup[0]=ctrl->error;
	
  ctrl->last_error=ctrl->error;//�����ϴ�ƫ��
  ctrl->error=ctrl->expect-ctrl->feedback;//������ȥ�����õ�ƫ��
	ctrl->dis_error=ctrl->error-ctrl->error_backup[ctrl->dis_error_gap_cnt-1];//΢��		ctrl->dis_error=ctrl->error-ctrl->last_error;//ԭʼ΢��
	
  if(ctrl->error_limit_flag==1)//ƫ���޷��ȱ�־λ
  {
    if(ctrl->error>= ctrl->error_limit_max)  ctrl->error= ctrl->error_limit_max;
    if(ctrl->error<=-ctrl->error_limit_max)  ctrl->error=-ctrl->error_limit_max;
  }
  /*******���ּ���*********************/
  if(ctrl->integral_separate_flag==1)//���ַ����־λ
  {
		//ֻ��ƫ��Ƚ�С��ʱ��������ֿ���
    if(ABS(ctrl->error)<=ctrl->integral_separate_limit)	 ctrl->integral+=ctrl->ki*ctrl->error;
  }
  else
  {
    ctrl->integral+=ctrl->ki*ctrl->error;
  }
	/*******�����޷�*********************/
	if(ctrl->integral>=ctrl->integral_limit_max)   ctrl->integral=ctrl->integral_limit_max;
	if(ctrl->integral<=-ctrl->integral_limit_max)  ctrl->integral=-ctrl->integral_limit_max;
	
	/*******���������*********************/
  ctrl->last_output=ctrl->output;//���ֵ����
  ctrl->output=ctrl->kp*ctrl->error//����
							+ctrl->integral//����
							+ctrl->kd*ctrl->dis_error;//΢��
	
	/*******������޷�*********************/
  if(ctrl->output>= ctrl->output_limit_max)  ctrl->output= ctrl->output_limit_max;
  if(ctrl->output<=-ctrl->output_limit_max)  ctrl->output=-ctrl->output_limit_max;
  /*******���������*********************/
  return ctrl->output;
}



float pid_control_dt_run(controller *ctrl,float dt)
{
  float _dt=0;
  get_systime(&ctrl->_time);
  _dt=ctrl->_time.period/1000.0f;
	if(_dt>1.05f*dt||_dt<0.95f*dt||isnan(_dt)!=0)   _dt=dt;
	if(_dt<0.0001f) return 0;

  /*******ƫ�����*********************/
	for(uint16_t i=19;i>0;i--)
	{
		ctrl->error_backup[i]=ctrl->error_backup[i-1];
	}
	ctrl->error_backup[0]=ctrl->error;
	
  ctrl->last_error=ctrl->error;//�����ϴ�ƫ��
  ctrl->error=ctrl->expect-ctrl->feedback;//������ȥ�����õ�ƫ��
	ctrl->dis_error=ctrl->error-ctrl->error_backup[ctrl->dis_error_gap_cnt-1];//΢��		ctrl->dis_error=ctrl->error-ctrl->last_error;//ԭʼ΢��

  if(ctrl->error_limit_flag==1)//ƫ���޷��ȱ�־λ
  {
    if(ctrl->error>= ctrl->error_limit_max)  ctrl->error= ctrl->error_limit_max;
    if(ctrl->error<=-ctrl->error_limit_max)  ctrl->error=-ctrl->error_limit_max;
  }
  /*******���ּ���*********************/
  if(ctrl->integral_separate_flag==1)//���ַ����־λ
  {
		//ֻ��ƫ��Ƚ�С��ʱ��������ֿ���
    if(ABS(ctrl->error)<=ctrl->integral_separate_limit)	 ctrl->integral+=ctrl->ki*ctrl->error*_dt;
  }
  else
  {
    ctrl->integral+=ctrl->ki*ctrl->error*_dt;
  }
	/*******�����޷�*********************/
	if(ctrl->integral>=ctrl->integral_limit_max)   ctrl->integral=ctrl->integral_limit_max;
	if(ctrl->integral<=-ctrl->integral_limit_max)  ctrl->integral=-ctrl->integral_limit_max;
	
	ctrl->dis_error_lpf=LPButterworth(ctrl->dis_error,&ctrl->lpf_buffer,&ctrl->lpf_params);
		
	/*******���������*********************/
  ctrl->last_output=ctrl->output;//���ֵ����
  ctrl->output=ctrl->kp*ctrl->error//����
							+ctrl->integral//����
							+ctrl->kd*ctrl->dis_error_lpf;//΢��
	
	/*******������޷�*********************/
  if(ctrl->output>= ctrl->output_limit_max)  ctrl->output= ctrl->output_limit_max;
  if(ctrl->output<=-ctrl->output_limit_max)  ctrl->output=-ctrl->output_limit_max;
  /*******���������*********************/
  return ctrl->output;
}


float pid_control_dt_yaw_run(controller *ctrl,float dt)
{
  float _dt=0;
  get_systime(&ctrl->_time);
  _dt=ctrl->_time.period/1000.0f;
	if(_dt>1.05f*dt||_dt<0.95f*dt||isnan(_dt)!=0)   _dt=dt;
	if(_dt<0.0001f) return 0;
	
  /*******ƫ�����*********************/
	for(uint16_t i=19;i>0;i--)
	{
		ctrl->error_backup[i]=ctrl->error_backup[i-1];
	}
	ctrl->error_backup[0]=ctrl->error;
	
  ctrl->last_error=ctrl->error;//�����ϴ�ƫ��
  ctrl->error=ctrl->expect-ctrl->feedback;//������ȥ�����õ�ƫ��
	ctrl->dis_error=ctrl->error-ctrl->error_backup[ctrl->dis_error_gap_cnt-1];//΢��		ctrl->dis_error=ctrl->error-ctrl->last_error;//ԭʼ΢��

  /***********************ƫ����ƫ���+-180����*****************************/
	if(ctrl->error<-180) ctrl->error=ctrl->error+360;
	if(ctrl->error>180)  ctrl->error=ctrl->error-360;

	
  if(ctrl->error_limit_flag==1)//ƫ���޷��ȱ�־λ
  {
    if(ctrl->error>= ctrl->error_limit_max)  ctrl->error= ctrl->error_limit_max;
    if(ctrl->error<=-ctrl->error_limit_max)  ctrl->error=-ctrl->error_limit_max;
  }
  /*******���ּ���*********************/
  if(ctrl->integral_separate_flag==1)//���ַ����־λ
  {
		//ֻ��ƫ��Ƚ�С��ʱ��������ֿ���
    if(ABS(ctrl->error)<=ctrl->integral_separate_limit)	 ctrl->integral+=ctrl->ki*ctrl->error*_dt;
  }
  else
  {
    ctrl->integral+=ctrl->ki*ctrl->error*_dt;
  }
	/*******�����޷�*********************/
	if(ctrl->integral>=ctrl->integral_limit_max)   ctrl->integral=ctrl->integral_limit_max;
	if(ctrl->integral<=-ctrl->integral_limit_max)  ctrl->integral=-ctrl->integral_limit_max;
	
	/*******���������*********************/
  ctrl->last_output=ctrl->output;//���ֵ����
  ctrl->output=ctrl->kp*ctrl->error//����
							+ctrl->integral//����
							+ctrl->kd*ctrl->dis_error;//΢��
	
	/*******������޷�*********************/
  if(ctrl->output>= ctrl->output_limit_max)  ctrl->output= ctrl->output_limit_max;
  if(ctrl->output<=-ctrl->output_limit_max)  ctrl->output=-ctrl->output_limit_max;
  /*******���������*********************/
  return ctrl->output;
}

