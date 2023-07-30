#include "Headfile.h"
#include "raspi_undercontrol.h"

static uint8_t Raspi_Head[2] = {0xFF, 0xFC}; // 数据帧头
static uint8_t Raspi_End[2] = {0xA1, 0xA2};  // 数据帧尾

uint8_t Raspi_Ctrl_Send_Databuf[50];    // 要发送给树莓派的数据缓冲区
/**
 * @brief 串口发送数据函数
 * @param buf 数据缓冲区的数组指针
 * @param cnt 数组指针的长度
 */
void Serial_Data_Send(uint8_t *buf, uint32_t cnt)
{
    UART_SendBytes(3, buf, cnt); // 用户移植时，重写此串口发送函数
}

/**
 * @brief 小车接收到树莓派状态显示
 */
void Car_Status_Tick(void)
{
    bling_set(&light_red, 500, 50, 0.2, 0, 0); // 红色  //用户移植时，重写此函数
}

union
{
    unsigned char floatByte[4];
    float floatValue;
} FloatUnion;

/**
 * @brief 将float数据转成4字节数据并存入指定地址
 * @param FloatValue float
 * @param Byte 数组
 * @param Subscript 指定从数组第几个元素开始写入
 *
 */
void Float2Byte(float *FloatValue, uint8_t *Byte, uint8_t Subscript)
{
    FloatUnion.floatValue = (float)2;
    if (FloatUnion.floatByte[0] == 0) // 小端模式
    {
        FloatUnion.floatValue = *FloatValue;
        Byte[Subscript] = FloatUnion.floatByte[0];
        Byte[Subscript + 1] = FloatUnion.floatByte[1];
        Byte[Subscript + 2] = FloatUnion.floatByte[2];
        Byte[Subscript + 3] = FloatUnion.floatByte[3];
    }
    else // 大端模式
    {
        FloatUnion.floatValue = *FloatValue;
        Byte[Subscript] = FloatUnion.floatByte[0];
        Byte[Subscript + 1] = FloatUnion.floatByte[1];
        Byte[Subscript + 2] = FloatUnion.floatByte[2];
        Byte[Subscript + 3] = FloatUnion.floatByte[3];
    }
}
/**
 * @brief 从指定地址将4字节数据转成float数据
 * @param Byte 数组
 * @param Subscript 指定从数组第几个元素开始写入
 * @param FloatValue float值
 */
void Byte2Float(uint8_t *Byte, uint8_t Subscript, float *FloatValue)
{
    FloatUnion.floatByte[0] = Byte[Subscript];
    FloatUnion.floatByte[1] = Byte[Subscript + 1];
    FloatUnion.floatByte[2] = Byte[Subscript + 2];
    FloatUnion.floatByte[3] = Byte[Subscript + 3];
    *FloatValue = FloatUnion.floatValue;
}

/**
 * @brief 用于ti板子要发送给树莓派的任务
 */
void Tidata_Tosend_Raspi(uint8_t mode)
{
    uint8_t _cnt = 0;
    uint8_t sum = 0;
    uint8_t i;
    Raspi_Ctrl_Send_Databuf[_cnt++] = Raspi_Head[0];
    Raspi_Ctrl_Send_Databuf[_cnt++] = Raspi_Head[1];
    Raspi_Ctrl_Send_Databuf[_cnt++] = Raspi_Ctrl_Send_Instruction_toRaspi + Instruction_Base_Address;
    Raspi_Ctrl_Send_Databuf[_cnt++] = 0;  // 有效数据长度
    Raspi_Ctrl_Send_Databuf[_cnt++] = mode;  // 有效数据

    Raspi_Ctrl_Send_Databuf[3] = _cnt - 4;  // 有效数据长度
    // 异或校验位
    for(i = 0; i < _cnt; i++) sum ^= Raspi_Ctrl_Send_Databuf[i];
    Raspi_Ctrl_Send_Databuf[_cnt++] = sum;
    Raspi_Ctrl_Send_Databuf[_cnt++] = Raspi_End[0];
    Raspi_Ctrl_Send_Databuf[_cnt++] = Raspi_End[1];
    Serial_Data_Send(Raspi_Ctrl_Send_Databuf, _cnt);
}

uint32_t Raspi_receive_fault_cnt = 0;
static uint8_t Raspi_Receivebuf[100];


/**
 * @brief 解析来自树莓派发送的数据
 * @param data 数据
 */
void Raspi_Data_Phrase_Prepare_Lite(uint8_t data)
{
    static uint8_t data_len = 0, data_cnt = 0;
    static uint8_t state = 0;

    if (state == 0 && data == Raspi_Head[0]) // 判断帧头1 0xFF
    {

        state = 1;
        Raspi_Receivebuf[0] = data;
    }
    else if (state == 1 && data == Raspi_Head[1]) // 判断帧头2 0xFC
    {


        state = 2;
        Raspi_Receivebuf[1] = data;
    }
    else if (state == 2 && data < 0XF1) // 功能字节，树莓派发送命令控制
    {
        state = 3;
        Raspi_Receivebuf[2] = data;


    }
    else if (state == 3 && data < 100) // 有效数据长度
    {
        state = 4;
        Raspi_Receivebuf[3] = data;
        data_len = data; // 有效数据长度
        data_cnt = 0;
    }
    else if (state == 4 && data_len > 0) // 数据接收
    {
        data_len--;
        Raspi_Receivebuf[4 + data_cnt++] = data;
        if (data_len == 0)
            state = 5;
    }
    else if (state == 5) // 异或校验位
    {
        state = 6;
        Raspi_Receivebuf[4 + data_cnt++] = data;
    }

    else if (state == 6 && data == Raspi_End[0]) // 帧尾0   0xA1
    {
        state = 7;
        Raspi_Receivebuf[4 + data_cnt++] = data;
    }
    else if (state == 7 && data == Raspi_End[1]) // 帧尾1 0xA2
    {
        state = 0;
        Raspi_Receivebuf[4 + data_cnt] = data;
        Raspi_Data_Phrase_Process_Lite(Raspi_Receivebuf, data_cnt + 5); // 数据解析
    }
    else
    {
        state = 0;
        Raspi_receive_fault_cnt++;
    }
}

Raspi_Ctrl_Procedure raspi_ctrl_procedure;

/**
 * @brief 树莓派控制参数初始化
*/
void Raspi_Ctrl_Procedure_Init(void)
{
    for(uint8_t i = 0; i < Instruction_Number_Max; i++)
    {
        raspi_ctrl_procedure.Instruture_Pending_Bit[i] = 0;
        raspi_ctrl_procedure.Task_Executing_Bit[i] = 0;
    }
}



void Raspi_Data_Phrase_Process_Lite(uint8_t *data_buf, uint8_t num) // 树莓派数据解析进程
{

    uint8_t _cnt = 0;
    uint8_t sum = 0;
    for(uint8_t i=0;i<(num-3);i++)  sum ^= *(data_buf+i);    // 异或校验
    if(!(sum==*(data_buf+num-3)))
        if(!(*(data_buf)==Raspi_Head[0]&&*(data_buf+1)==Raspi_Head[1]))         return;//判断帧头
        if(!(*(data_buf+num-2)==Raspi_End[0]&&*(data_buf+num-1)==Raspi_End[1])) return;//帧尾校验 
    
 
    raspi_ctrl_procedure.instruture = *(data_buf+2);        // 第三位为指令

    switch (raspi_ctrl_procedure.instruture)
    {
    case Raspi_Ctrl_Speed_Control + Instruction_Base_Address: // 速度控制指令
    {
        _cnt = 0;
        raspi_ctrl_procedure.speed_byte_buf[_cnt++] = *(data_buf + _cnt + 4);
        raspi_ctrl_procedure.speed_byte_buf[_cnt++] = *(data_buf + _cnt + 4);
        raspi_ctrl_procedure.speed_byte_buf[_cnt++] = *(data_buf + _cnt + 4);
        raspi_ctrl_procedure.speed_byte_buf[_cnt++] = *(data_buf + _cnt + 4);
        Byte2Float(raspi_ctrl_procedure.speed_byte_buf, 0, &raspi_ctrl_procedure.speed);
        raspi_ctrl_procedure.Instruture_Pending_Bit[Raspi_Ctrl_Speed_Control] = 1;
        Car_Status_Tick();
    }
    break;
    case Raspi_Ctrl_Distance_Control + Instruction_Base_Address: // 距离控制指令
    {
        _cnt = 0;
        raspi_ctrl_procedure.distance_byte_buf[_cnt++] = *(data_buf + _cnt + 4);
        raspi_ctrl_procedure.distance_byte_buf[_cnt++] = *(data_buf + _cnt + 4);
        raspi_ctrl_procedure.distance_byte_buf[_cnt++] = *(data_buf + _cnt + 4);
        raspi_ctrl_procedure.distance_byte_buf[_cnt++] = *(data_buf + _cnt + 4);
        Byte2Float(raspi_ctrl_procedure.distance_byte_buf, 0, &raspi_ctrl_procedure.distance);
        raspi_ctrl_procedure.Instruture_Pending_Bit[Raspi_Ctrl_Distance_Control] = 1;
        Car_Status_Tick();
    }
    break;
    case Raspi_Ctrl_Contrarotate_90 + Instruction_Base_Address: // 左转90度
    {
        raspi_ctrl_procedure.Instruture_Pending_Bit[Raspi_Ctrl_Contrarotate_90] = 1;
        Car_Status_Tick();
    }
    break;
    case Raspi_Ctrl_Clockwise_Rotation_90 + Instruction_Base_Address: // 右转90度
    {
        raspi_ctrl_procedure.Instruture_Pending_Bit[Raspi_Ctrl_Clockwise_Rotation_90] = 1;
        Car_Status_Tick();
    }
    break;

   
    case Raspi_Ctrl_OPen_Loop_Output_Pwm + Instruction_Base_Address: // 开环输出pwm
    {
        raspi_ctrl_procedure.left_pwm = *(data_buf + 4) << 8 | *(data_buf + 5);
        raspi_ctrl_procedure.right_pwm = *(data_buf + 6) << 8 | *(data_buf + 7);
        raspi_ctrl_procedure.Instruture_Pending_Bit[Raspi_Ctrl_OPen_Loop_Output_Pwm] = 1;
        Car_Status_Tick();
        
    }
    break;
    default:
    {
        for (uint8_t i = 0; i < Instruction_Number_Max; i++)
        {
            raspi_ctrl_procedure.Instruture_Pending_Bit[i] = 0;
        }
    }
    }
}

/**
 * @brief 树莓派指令分配函数
*/
void Raspi_Ctrl_Instruction_Dispatch(void)
{
    static uint8_t i;
    static uint8_t Executing_flag = 0;
    if (raspi_ctrl_procedure.Instruture_Pending_Bit[Raspi_Ctrl_Speed_Control])  // 如果是待解决的话
    {
        trackless_output.unlock_flag = UNLOCK;
        sdk_work_mode = Speed_Control;
        raspi_ctrl_procedure.Instruture_Pending_Bit[Raspi_Ctrl_Speed_Control] = 0;
        raspi_ctrl_procedure.Task_Executing_Bit[Raspi_Ctrl_Speed_Control] = 1; // 挂起正在执行
        subtask_thread_reset(Speed_Control);                                   // 复位子任务线程
    }
    else if (raspi_ctrl_procedure.Instruture_Pending_Bit[Raspi_Ctrl_Distance_Control])
    {
        trackless_output.unlock_flag = UNLOCK;
        sdk_work_mode = Distance_Control;
        raspi_ctrl_procedure.Instruture_Pending_Bit[Raspi_Ctrl_Distance_Control] = 0;
        raspi_ctrl_procedure.Task_Executing_Bit[Raspi_Ctrl_Distance_Control] = 1; // 挂起正在执行
        subtask_thread_reset(Distance_Control);                                   // 复位子任务线程
    }
    else if (raspi_ctrl_procedure.Instruture_Pending_Bit[Raspi_Ctrl_Contrarotate_90])
    {
        trackless_output.unlock_flag = UNLOCK;
        sdk_work_mode = Contrarotate_90;
        raspi_ctrl_procedure.Instruture_Pending_Bit[Raspi_Ctrl_Contrarotate_90] = 0;

        raspi_ctrl_procedure.Task_Executing_Bit[Raspi_Ctrl_Contrarotate_90] = 1; // 挂起正在执行
        subtask_thread_reset(Contrarotate_90);                                   // 复位子任务线程
    }
    else if (raspi_ctrl_procedure.Instruture_Pending_Bit[Raspi_Ctrl_Clockwise_Rotation_90])
    {
        trackless_output.unlock_flag = UNLOCK;
        sdk_work_mode = Clockwise_Rotation_90;
        raspi_ctrl_procedure.Instruture_Pending_Bit[Raspi_Ctrl_Clockwise_Rotation_90] = 0;

        raspi_ctrl_procedure.Task_Executing_Bit[Raspi_Ctrl_Clockwise_Rotation_90] = 1; // 挂起正在执行
        subtask_thread_reset(Clockwise_Rotation_90);                                   // 复位子任务线程
    }
    else if(raspi_ctrl_procedure.Instruture_Pending_Bit[Raspi_Ctrl_OPen_Loop_Output_Pwm])
    {
        trackless_output.unlock_flag = UNLOCK;
        raspi_ctrl_procedure.Instruture_Pending_Bit[Raspi_Ctrl_OPen_Loop_Output_Pwm] = 0;
        raspi_ctrl_procedure.Task_Executing_Bit[Raspi_Ctrl_OPen_Loop_Output_Pwm] = 1;
    }
    else
    {
        Executing_flag = 0;
        for(i = 0; i < Instruction_Number_Max; i++)
        {
            if(raspi_ctrl_procedure.instruture == Raspi_Ctrl_Speed_Control)
            {
                Executing_flag = 1;
                break;
            }
            if(raspi_ctrl_procedure.Task_Executing_Bit[i])  // 如果有任务正在执行
            {
                Executing_flag = 1;
            }
        }
        if(Executing_flag == 0) trackless_output.unlock_flag = LOCK; // 没任务执行则锁电机
        
    }
}



/**
 * @brief Ti板给树莓派发送应答和完成标志位
 * 
*/
void Raspi_Ctrl_Send_State(void)
{
    uint8_t _cnt = 0;
    uint8_t sum = 0;
    uint8_t i;
    Raspi_Ctrl_Send_Databuf[_cnt++] = Raspi_Head[0];
    Raspi_Ctrl_Send_Databuf[_cnt++] = Raspi_Head[1];
    Raspi_Ctrl_Send_Databuf[_cnt++] = Raspi_Ctrl_Send_State_Data + Instruction_Base_Address;
    Raspi_Ctrl_Send_Databuf[_cnt++] = 0;  // 有效数据长度
    Raspi_Ctrl_Send_Databuf[_cnt++] = subtask_finish_flag[Speed_Control];  // 速度控制完成标志位
    Raspi_Ctrl_Send_Databuf[_cnt++] = subtask_finish_flag[Distance_Control];
    Raspi_Ctrl_Send_Databuf[_cnt++] = subtask_finish_flag[Contrarotate_90];
    Raspi_Ctrl_Send_Databuf[_cnt++] = subtask_finish_flag[Clockwise_Rotation_90];
    
    Raspi_Ctrl_Send_Databuf[3] = _cnt - 4;
    // 异或校验位
    for(i = 0; i < _cnt; i++) sum ^= Raspi_Ctrl_Send_Databuf[i];
    Raspi_Ctrl_Send_Databuf[_cnt++] = sum;
    Raspi_Ctrl_Send_Databuf[_cnt++] = Raspi_End[0];
    Raspi_Ctrl_Send_Databuf[_cnt++] = Raspi_End[1];
    Serial_Data_Send(Raspi_Ctrl_Send_Databuf, _cnt);
}


