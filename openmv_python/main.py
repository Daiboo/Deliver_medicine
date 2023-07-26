#main.py -- put your code here!
import cpufreq
import pyb
import sensor,image, time,math
from pyb import LED,Timer,UART

sensor.reset()                      # Reset and initialize the sensor.
sensor.set_pixformat(sensor.RGB565) # Set pixel format to RGB565 (or GRAYSCALE)
sensor.set_framesize(sensor.QQQVGA)  # Set frame size to 80x60
sensor.skip_frames(time = 2000)     #延时跳过一些帧，等待感光元件变稳定
sensor.set_auto_gain(False)          #黑线不易识别时，将此处写False
sensor.set_auto_whitebal(False)
clock = time.clock()                # Create a clock object to track the FPS.

#sensor.set_auto_exposure(True, exposure_us=5000) # 设置自动曝光sensor.get_exposure_us()

uart=UART(3,256000)  # 串口


# -------------------------------------------预变量--------------------------------------------
#--------------------任务类型---------------------
class Task_Type:
    Number_recognition_inbegin_task = 0x01  # 用于起初识别数字的任务
    Tracking_task = 0x02    # 循迹任务
    Number_recognition_intrack_task = 0x03  # 用于十字路口识别数字的任务
task_type = Task_Type()

#-------------------图像参数----------------------




# ---------------------------------要传输的目标数据---------------------------------------------

class target_data_Data(object):   # 要传输的目标数据
    x=0          #int16_t   循迹信息
    flag=0       #uint8_t   循迹成功标志位
    fps=0        #uint8_t   fps
    camera_id=0  # 摄像机id 


target_data=target_data_Data()  # 目标检测信息


# -----------------------------------------rgb闪烁------------------------------------------------
class rgb(object):  # rgb类
    def __init__(self):
        self.red=LED(1)
        self.green=LED(2)
        self.blue=LED(3)

rgb=rgb()  # led灯

def time_callback(info):
    rgb.red.toggle()

timer=Timer(2,freq=2)  # 计时器2，频率为4hz,0.25s
timer.callback(time_callback)  # 0.25s调用一次





# -----------------------------------------任务控制------------------------------------------------

class Task_Ctrl(object):  # 用于记录和更改openmv的任务
    task = task_type.Number_recognition_inbegin_task   # 默认数字识别任务

task_ctrl = Task_Ctrl()



# --------------------------------------串口数据读取-----------------------------------------------
class uart_buf_prase(object):  # 用于记录数据，有效数据长度，数据包长度
    uart_buf = []
    _data_len = 0
    _data_cnt = 0
    state = 0         # 状态机状态

R=uart_buf_prase()

#串口数据解析
def Receive_Anl(data_buf,num):
    #和校验
    sum = 0
    i = 0
    while i<(num-1): # 0-num-2
        sum = sum + data_buf[i]
        i = i + 1
    sum = sum%256 #求余
    if sum != data_buf[num-1]:
        return
    #和校验通过
    if data_buf[2]==0xA0:   # 第三位  0xc0+mode
        #设置模块工作模式
        task_ctrl.task = data_buf[4]  # 获取任务类型
        print(task_ctrl.task)
        print("Set work mode success!")


def uart_data_prase(buf):  # 读取数据状态机
    if R.state==0 and buf==0xFF:#帧头1
        R.state=1
        R.uart_buf.append(buf)
    elif R.state==1 and buf==0xFE:#帧头2
        R.state=2
        R.uart_buf.append(buf)
    elif R.state==2 and buf<0xFF:#功能字
        R.state=3
        R.uart_buf.append(buf)
    elif R.state==3 and buf<50:#数据长度小于50
        R.state=4
        R._data_len=buf  #有效数据长度
        R._data_cnt=buf+5#总数据长度
        R.uart_buf.append(buf)
    elif R.state==4 and R._data_len>0:#存储对应长度数据
        R._data_len=R._data_len-1
        R.uart_buf.append(buf)
        if R._data_len==0:
            R.state=5
    elif R.state==5:
        R.uart_buf.append(buf)
        R.state=0
        Receive_Anl(R.uart_buf,R.uart_buf[3]+5)
#        print(R.uart_buf)
        R.uart_buf=[]#清空缓冲区，准备下次接收数据
    else:
        R.state=0
        R.uart_buf=[]#清空缓冲区，准备下次接收数据


def uart_data_read():  # 串口数据读取并解析
    buf_len=uart.any()
    for i in range(0,buf_len):
        uart_data_prase(uart.readchar())

# --------------------------------------串口数据发送----------------------------------------------


target_data.camera_id=0x01  # 摄像机id
HEADER=[0xFF,0xFC]   # 帧头

def package_blobs_data(task):
    #数据打包封装
    data=bytearray([HEADER[0],HEADER[1],task,0x00,  # 第三位发送任务，第四位发送有效数据个数
                   target_data.x>>8,target_data.x,        #将整形数据拆分成两个8位  x，y是int16_t
                   target_data.flag,                 #数据有效标志位
                   target_data.fps,      #数据有效标志位
                   0x00])
    #数据包的长度
    data_len=len(data)
    data[3]=data_len-5#有效数据的长度 长度减去5,只剩下有用的数据
    #最后一位和校验
    sum=0
    for i in range(0,data_len-1): 
        sum=sum+data[i]
    data[data_len-1]=sum 
    #返回打包好的数据
    return data




# -------------------------------------16路循迹任务--------------------------------------

# 图像80*60 一共16个
track_roi=[(0,25,5,10),
           (5,25,5,10),
           (10,25,5,10),
           (15,25,5,10),
           (20,25,5,10),
           (25,25,5,10),
           (30,25,5,10),
           (35,25,5,10),
           (40,25,5,10),
           (45,25,5,10),
           (50,25,5,10),

           (55,25,5,10),
           (60,25,5,10),
           (65,25,5,10),
           (70,25,5,10),
           (75,25,5,10)]


thresholds =(8, 30, -30, 30, -30, 30)  # Lab阈值
class linedata(object):  # 用于保存检测结果
    bit0=0
    bit1=0
    bit2=0
    bit3=0
    bit4=0
    bit5=0
    bit6=0
    bit7=0
    bit8=0
    bit9=0
    bit10=0
    bit11=0
    bit12=0
    bit13=0
    bit14=0
    bit15=0

lines=linedata()
hor_bits=['0','0','0','0','0','0','0','0','0','0','0','0','0','0','0','0']  # 检测巡线

def findtrack():   # 16路循迹任务

    target_data.flag = 0
    target_data.x=0
    img=sensor.snapshot()


    # 寻找巡线
    for i in range(0,16):
        hor_bits[i]=0
        blobs=img.find_blobs([thresholds],roi=track_roi[i],merge=True,margin=10)  # 寻找色块
        for b in blobs:
            hor_bits[i]=1

    target_data.flag = 1   # 识别成功标志位
    for k in range(0,16):
        if  hor_bits[k]:
            target_data.x=target_data.x|(0x01<<(15-k))  # 移动15位就为：1000 0000 0000 0000
            img.draw_circle(int(track_roi[k][0]+track_roi[k][2]*0.5),int(track_roi[k][1]+track_roi[k][3]*0.5),1,(255,0,0))
    for rec in track_roi:
        img.draw_rectangle(rec, color=(0,0,255))#绘制出roi区域


    print(target_data.x)  # 打印循迹结果



# ------------------------------------------主程序------------------------------------------

task_ctrl.task = task_type.Number_recognition_inbegin_task
while True:
    clock.tick()

    uart_data_read()  # 读取单片机发来的指令

    if task_ctrl.task == task_type.Number_recognition_inbegin_task:
        pass     # todo 起初数字识别任务
    elif task_ctrl.task == task_type.Tracking_task:
        findtrack()
    elif task_ctrl.task == task_type.Number_recognition_intrack_task:
        pass    # todo 赛道数字识别任务

    uart.write(package_blobs_data(task_ctrl.task))
    
    #计算fps
    target_data.fps = (int)(clock.fps())

