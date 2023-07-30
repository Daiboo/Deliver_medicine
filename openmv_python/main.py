#main.py -- put your code here!
import cpufreq
import pyb
import sensor,image, time,math
from pyb import LED,Timer,UART
from image import SEARCH_EX, SEARCH_DS
import os, tf, uos, gc

sensor.reset()                      # Reset and initialize the sensor.
sensor.set_pixformat(sensor.RGB565) # Set pixel format to RGB565 (or GRAYSCALE)
sensor.set_framesize(sensor.QQQVGA)  # Set frame size to 80x60
sensor.skip_frames(time = 2000)     #延时跳过一些帧，等待感光元件变稳定
sensor.set_auto_gain(False)          #黑线不易识别时，将此处写False
sensor.set_auto_whitebal(False)
sensor.set_hmirror(True)            #设置镜像
sensor.set_vflip(True)              #设置翻转
sensor.set_contrast(3)
sensor.set_gainceiling(16)
sensor.set_pixformat(sensor.GRAYSCALE)
import lcd

# lcd.init()

clock = time.clock()                # Create a clock object to track the FPS.

#sensor.set_auto_exposure(True, exposure_us=5000) # 设置自动曝光sensor.get_exposure_us()

uart=UART(3,256000)  # 串口




# -------------------------------------------预变量--------------------------------------------
#--------------------任务类型---------------------
class Task_Type:
    Number_recognition_inbegin_task = 0xA1  # 用于起初识别数字的任务
    Tracking_task = 0xA2    # 循迹任务
    Number_recognition_intrack_task = 0xA3  # 用于十字路口识别数字的任务
task_type = Task_Type()

#-------------------图像参数----------------------

# --------- Target 数字 --------------
TARGET_NUM = 0



# ---------------------------------要传输的目标数据---------------------------------------------

class Target_Data_GraySensor(object):   # 要传输的目标数据
    x=0          #int16_t   循迹信息
    cross=0      #uint8_t   是否到达十字路口或T路口
    flag=0       #uint8_t   循迹成功标志位
    fps=0        #uint8_t   fps
    camera_id=0  # 摄像机id

class Target_Data_NeedTodo(object):   # 要传输的目标数据
    x = 0b000   #uint8_t    3位二进制：100左转，000直走，001右转

class Target_Data_Begin(object):   # 要传输的目标数据
    flag = 0b0   #uint8_t     # 是否成功读取


target_data_graysensor=Target_Data_GraySensor()  # 目标检测信息
target_data_needtodo = Target_Data_NeedTodo()   # 左右转弯信息
target_data_begin = Target_Data_Begin()     # 开始时信息读取

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
    print(sum)
    if sum != data_buf[num-1]:
        return
    #和校验通过
    if data_buf[2]==0xA0:
        #设置模块工作模式
        task_ctrl.task = data_buf[4]  # 获取任务类型
        print(task_ctrl.task)
        print("Set work mode success!")


def uart_data_prase(buf):  # 读取数据状态机
    if R.state==0 and buf==0xFF:#帧头1 0xFF
        R.state=1
        R.uart_buf.append(buf)
    elif R.state==1 and buf==0xFE:#帧头2  # 0xFE
        R.state=2
        R.uart_buf.append(buf)
    elif R.state==2 and buf<0xFF:#功能字  0xA0
        R.state=3
        R.uart_buf.append(buf)
    elif R.state==3 and buf<50:#数据长度小于50  2
        R.state=4
        R._data_len=buf  #有效数据长度
        R._data_cnt=buf+5#总数据长度
        R.uart_buf.append(buf)
    elif R.state==4 and R._data_len>0:#存储对应长度数据  task 0
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


target_data_graysensor.camera_id=0x01  # 摄像机id
HEADER=[0xFF,0xFC]   # 帧头

def package_blobs_data(task):
    #数据打包封装
    if task == task_type.Tracking_task:
        data=bytearray([HEADER[0],HEADER[1],task,0x00,  # 第三位发送任务，第四位发送有效数据个数
                   target_data_graysensor.x>>8,target_data_graysensor.x,        #将整形数据拆分成两个8位  x，y是int16_t
                   target_data_graysensor.cross,    # 是否到达交叉路口
                   target_data_graysensor.flag,                 #数据有效标志位
                   target_data_graysensor.fps,      #数据有效标志位
                   0x00])
    elif task == task_type.Number_recognition_inbegin_task:
        data = bytearray([HEADER[0],HEADER[1],task,0x00,
        target_data_begin.flag,
#        target_data_graysensor.fps,      #数据有效标志位
        0x00])
    elif task == task_type.Number_recognition_intrack_task:
        data = bytearray([HEADER[0],HEADER[1],task,0x00,
        target_data_needtodo.x,
#        target_data_graysensor.fps,      #数据有效标志位
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

# ---------------- Read Template For Digit -----------------------------
temp_list = [x + 1 for x in range(8)]
temp_list = [os.listdir(f'/{x}') for x in temp_list]
temp_feats = [[] for x in range(8)]
for i, temp_list_n in enumerate(temp_list):
    n_feat = []
    for temp_file in temp_list_n:
        n_feat.append(image.Image(f'/{i + 1}/{temp_file}'))
    temp_feats[i] = n_feat
    print(f"Loadded: {i + 1} Features!")

# --------------- Read Template For T And Shizi ------------------------
T1 = image.Image('/T1.pgm')
T2 = image.Image('/T2.pgm')
S1 = image.Image('/S1.pgm')
S2 = image.Image('/S2.pgm')
print('Loadded: T And S Features!')

# --------------- Read Template For InBegin Task ------------------------
begin_feats = []
for i in range(8):
    begin_feats.append(image.Image(f'/FSB/{i + 1}.pgm'))
print('Loadded: InBegin Features!')
# -------------------------------------16路循迹任务--------------------------------------
# -------------- 生成 ROI，自适应分辨率 ------------------
img_temp = sensor.snapshot()

# Image dimensions
img_width = img_temp.width()
img_height = img_temp.height()

# Number of ROIs
num_rois = 8

# ROI parameters
roi_width = 5 * (img_width // 80)
roi_height = 8 * (img_height // 60)

# List to store the ROIs
track_roi = []

# Calculate the horizontal and vertical gaps between ROIs
horizontal_gap = (img_width - (16 * roi_width)) // (16 - 1)
vertical_position = 40 * (img_height // 60)

offset_roi = 16 - num_rois

offset_fix_pix = 3

# Generate the ROIs
for i in range(num_rois):
    roi_x = i * (roi_width + horizontal_gap) + offset_fix_pix
    roi_y = vertical_position
    roi = [roi_x, roi_y, roi_width, roi_height]
    track_roi.append(roi)


if offset_roi != 0:
    # Need Offset the ROI
    for i in range(num_rois):
        track_roi[i][0] += roi_width * (offset_roi // 2)
#print(track_roi)
#mid1, mid2 = track_roi[7], track_roi[8]
#tar_roi = ((mid1[0] + mid2[0]) // 2, vertical_position + roi_height, roi_width, roi_height)
# -------------------- 结束ROI生成--------------------------

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
    target_data_graysensor.cross = 0
    target_data_graysensor.flag = 0
    target_data_graysensor.x=0
    img=sensor.snapshot()

    if find_cross(img): target_data_graysensor.cross = 1

    # 寻找巡线
    for i in range(0,num_rois):
        hor_bits[i]=0
#        print(track_roi[i])
        blobs=img.find_blobs([thresholds],roi=track_roi[i],merge=True,margin=10)  # 寻找色块
        for b in blobs:
            hor_bits[i]=1

    target_data_graysensor.flag = 1   # 识别成功标志位
    for k in range(0,num_rois):
        if  hor_bits[k]:
            target_data_graysensor.x=target_data_graysensor.x|(0x01<<(15-k))  # 移动15位就为：1000 0000 0000 0000
            img.draw_circle(int(track_roi[k][0]+track_roi[k][2]*0.5),int(track_roi[k][1]+track_roi[k][3]*0.5),1,(255,0,0))

    for iii in range(num_rois):
        rec = track_roi[iii]
        img.draw_rectangle(rec, color=(0,0,255))#绘制出roi区域


    print(target_data_graysensor.x)
    # lcd.display(img, x_scale=1.45, y_scale=1.45)

cross_range_x = 20

# -------------- Find T And Shi -----------------------
def find_cross(sensor_img) -> bool:
    for t in [T1, T2, S1, S2]:
        r = sensor_img.find_template(t, 0.9, step=1,search=SEARCH_EX,
                              roi=(32,0,15,15))
        if r:
            sensor_img.draw_rectangle(r)

            x_m = r[0] + r[2] // 2
            y_m = r[1] + r[3] // 2
            if (x_m in range(img_width // 2 - cross_range_x // 2, img_width // 2 + cross_range_x // 2)) and (y_m in range(2, 2 + 4)):
#                print(True)
                return True
#            print(i + 1, r) #打印模板名字



# -------------------------------- 读取模板信息 -------------------
def intrack_number_recognition():  # 途中数字识别任务
    img = sensor.snapshot()
    # lcd.display(img, x_scale=1.45, y_scale=1.45)
    target_data_needtodo.x = 0b000  # 无效
    for i, n_feat in enumerate(temp_feats):
        if TARGET_NUM != 0 and i != TARGET_NUM: continue
        for t in n_feat:
            r = img.find_template(t, 0.65, step=1,search=SEARCH_EX,
                                  roi=(0,0,80,40))
            if r:
                img.draw_rectangle(r)
                # lcd.display(img, x_scale=1.45, y_scale=1.45)
                n_xpoi = r[0] + (r[2] // 2)
                if n_xpoi < 40:
                    target_data_needtodo.x = 0b100
                else:
                    target_data_needtodo.x = 0b001
                print(i + 1, r) #打印模板名字
                break

def inbegin_number_recognition():  # 起初数字识别任务
    img = sensor.snapshot()
    # lcd.display(img, x_scale=1.45, y_scale=1.45)
    target_data_begin.flag = 0b0
    for i, t in enumerate(begin_feats):
        r = img.find_template(t, 0.65, step=1,search=SEARCH_EX)
#        print(i + 1, t)

        if r:
            n_time = time.time()
            img.draw_rectangle(r)
            # lcd.display(img, x_scale=1.45, y_scale=1.45)
            while(True):
                img = sensor.snapshot()
                r = img.find_template(t, 0.65, step=1,search=SEARCH_EX)
                if not r: break
                img.draw_rectangle(r)
                # lcd.display(img, x_scale=1.45, y_scale=1.45)
                if time.time() - n_time > 0.5:
                    print(True)
                    target_data_begin.flag = 0b1    # 成功读取
                    TARGET_NUM = i + 1
                    return
            print(i + 1, r) #打印模板名字

# ------------------------------------------主程序------------------------------------------

task_ctrl.task = task_type.Tracking_task

while True:
    clock.tick()

    uart_data_read()  # 读取单片机发来的指令

    if task_ctrl.task == task_type.Number_recognition_inbegin_task:
        inbegin_number_recognition()
    elif task_ctrl.task == task_type.Tracking_task:
        findtrack()
    elif task_ctrl.task == task_type.Number_recognition_intrack_task:
        intrack_number_recognition()

    uart.write(package_blobs_data(task_ctrl.task))
    target_data_graysensor.fps = int(clock.fps())
    print(f'fps = {clock.fps()}')
