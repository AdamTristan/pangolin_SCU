import cv2
import numpy as np
import serial  # python串口库
import serial.tools.list_ports
import time
import math
from skimage import morphology

# 加个串口名数组用于选择
COM_Name_Dic = {'NanoLidarCom': "/dev/ttyUSB0", 'NanoMpuCom': "/dev/ttyTHS1",
                'PCLidarCom': "COM8", 'PCMpuVirtualCom': "COM20"}
# 串口参数（仅调试时调整）
lidar_com_name = COM_Name_Dic['PCLidarCom']  # 雷达模块串口号,字符串格式。换不同USB接口这个名称要改。
lidar_baudrate = 115200  # 根据对方波特率改或者让对方改成这个。
mpu_com_name = COM_Name_Dic['PCMpuVirtualCom']
mpu_baudrate = 115200
name_lenth = 4  # 12为nano串口名长度，4为电脑串口长度
# Hough参数（lagori直径大小：500mm、425mm、350mm、275mm 和200mm）（关键参数，认真调整）
# ——需调整—— #
DP = 1.5  # 非常重要但却容易被忽视的可调参数！越大越容易识别到圆。最小值为1。##————核心—————##
Param2 = 22  # 它越小，可以检测到更多的假圆。它越大，能通过检测的圆就更加接近完美的圆形。##————核心—————##
# ——-----—— #
Param1 = 50  # 传递给 canny 边缘检测算子的高阀值，而低阀值为高阀值的一半。一般固定为50就行。
MinDist = 200  # 两个圆心的最短距离，两个lagori最近为100+137.5=237.5，留有裕量。
MinRadius = 50  # 圆最小的半径，lagori最小半径为100，留有裕量
MaxRadius = 285  # 圆最大的半径，lagori最大半径为250，留有裕量
# 雷达扫描区域限制(限定好区域，可以进一步缩小图幅大小）
# ——需调整—— #
right_bound = 60  # 待调整参数
Max_Polard = 500  # 搜寻范围的最大半径，待调整（500有点小了点似乎）
# ——-----—— #
left_bound = 360 - right_bound  # 不需调整
# 图幅大小(自动确定最佳图幅，不需调整。不过，一种可能更快的方式是不创建图，而是导入图。不清楚创建大背景图（本质是创建numpy数组）是不是很快。)
img_xsize = 2 * int(Max_Polard * math.sin(right_bound * math.pi / 180))
img_ysize = Max_Polard
# 绘点大小（待调整）
point_r = 17  # 所绘制点的半径大小（重要参数，值得调整，是大还是小？其大小直接决定了精度上限。跟据采用的算法的不同，该参数极小值不同。）
# 误差纠正参数（待调整。雷达距车中心x、y偏移量，单位mm）
Ly = 3814
Lx = 1836


# 定义雷达控制类：雷达串口配置与连接、雷达启动/停止、测量角度/距离的函数。
class LidarControl(object):
    # 初始化雷达类时必须强制配置好雷达串口(包括串口号）
    def __init__(self, port, baudrate, bytesize, stopbits, timeout, parity):
        # 初始化未配置的串口实例
        self.LidarCom = serial.Serial()
        # 强制初始化串口参数
        self.port = port
        self.baudrate = baudrate
        self.bytesize = bytesize
        self.stopbits = stopbits
        self.timeout = timeout
        self.parity = parity
        # 新定义的属性（用于class的全局变量）
        self.count = 0  # 初始化计数值count为0.
        self.angle = 0
        self.distan = 0
        self.distance = 0
        self.peri_flag = False
        self.lidar_open = False

    # 搜索并连接雷达串口（同时实例化）
    def LidarSerial_Connect(self):
        while True:  # 没连接到激光模块的串口则一直连接
            port_list = list(serial.tools.list_ports.comports())  # 搜索可用串口
            if len(port_list) == 0:  # 没有可用串口
                print("No Useful COM\r\n")
            else:
                for port in port_list:
                    port = str(port)
                    print(port)
                    print("try:", port[0:name_lenth])
                    # 如果找到，则开始串口配置
                    if port[0:name_lenth] == self.port:
                        # 激光串口实例化(6个必备参数）
                        self.LidarCom.port = self.port
                        self.LidarCom.baudrate = self.baudrate
                        self.LidarCom.bytesize = self.bytesize
                        self.LidarCom.stopbits = self.stopbits
                        self.LidarCom.timeout = self.timeout
                        self.LidarCom.parity = self.parity
                        # 关闭硬件流控
                        self.LidarCom.dtr = False
                        # 保险地写一个开启串口的命令
                        self.LidarCom.open()
                        if self.LidarCom.is_open:
                            print("Successfully Connected\r\n")
                            return 0  # 记得跳出循环(不能用break，因为break只是跳出for，而无法跳出while）
                        else:
                            print("Can't Connect\r\n")
                    else:
                        print("Incorrect COM\r\n")

    # 关闭雷达串口
    def LidarSerial_Close(self):
        self.LidarCom.close()

    # 开启雷达扫描
    def LidarStart(self):
        """由产品手册得知，往模块发送0xA5,0x20即可开启雷达"""
        print("尝试开启扫描")
        while True:
            self.LidarCom.flushInput()  # 清除接收区数据
            self.LidarCom.flushOutput()  # 清除发送区数据
            # 发送开始扫描命令
            self.LidarCom.write(b'\xA5')
            self.LidarCom.write(b'\x20')
            time.sleep(0.2)
            if self.LidarCom.inWaiting():  # 如果接收到数据，说明已经启动扫描了。
                self.lidar_open = True
                # 如果流控开启，则需要关闭，否则电机不转不会扫描。
                if self.LidarCom.dtr:
                    self.LidarCom.dtr = False
                print("扫描开始")
                break
            else:
                print("重新尝试发送开启扫描命令")

    # 关闭雷达扫描
    def LidarStop(self):
        """由产品手册得知，往模块发送0xA5,0x25即可关闭激光"""
        while True:
            self.LidarCom.flushOutput()  # 清除发送区数据
            # 发送停止扫描命令
            self.LidarCom.write(b'\xA5')
            self.LidarCom.write(b'\x25')
            # 清除当前接收缓存区数据
            self.LidarCom.flushInput()
            time.sleep(0.2)
            if self.LidarCom.inWaiting():  # 如果接收到数据，说明没有停止扫描。
                print("重新尝试发送停止扫描命令")
            else:
                self.lidar_open = False
                # 开启流控以使电机停转。
                self.LidarCom.dtr = True
                print("扫描停止")
                break

    # 这个是要放在循环里的.但是我们不必在意循环的控制，只要一直循环即可，然后对于返回的数据进行非空判断，非空则进行处理，否则继续循环。
    def LidarMeasure(self):
        """根据产品手册进行数据解析"""
        if self.LidarCom.inWaiting() > 0:  # 如果接受缓存区有数据
            res = self.LidarCom.read(1)  # 读取一个字节byte(并不是默认读取字符串）
            # ————————————————核心数据解析——————————————————————
            if self.peri_flag:  # 判断是否开启新的周期
                self.count = self.count + 1
                if self.count > 4:  # 周期结束
                    self.count = 0
                    self.peri_flag = False
                # if self.count == 1:  # 第二位是奇偶校验位，一般不需校验，可以忽略
                #     if ~(res[0] & 0x01):
                #         self.peri_flag=0
                #         self.count = 0
                if self.count == 2:  # 第三位是角度信息
                    self.angle = res[0] << 1  # 解算角度
                if self.count == 3:  # 第四位是距离信息之一
                    self.distan = res[0] >> 2
                if self.count == 4:  # 第五位是距离信息之二
                    self.distance = res[0] << 6 | self.distan  # 解算距离
            if res[0] == 0x3E:  # 周期开始（第一位是周期开始标志位）
                self.peri_flag = True

            # 表明进行完一次完整的解算，可以进行返回值了。
            if self.count == 4:
                return self.angle, self.distance  # 只有这种情况才返回非空的信息
            # 若不为4，则表明没有完成完整的解算，返回none。
            else:
                return None, None

        # 若没收到数据，则返回none
        else:
            return None, None


class ComCreator(serial.Serial):
    # 强制配置好串口(包括串口号）
    def __init__(self, port, baudrate, bytesize, stopbits, timeout, parity):
        super().__init__()  # 继承
        # 强制初始化串口参数
        self.newport = port
        self.newbaudrate = baudrate
        self.newbytesize = bytesize
        self.newstopbits = stopbits
        self.newtimeout = timeout
        self.newparity = parity

    # 搜索并连接串口
    def Serial_Connect(self):
        self.port = self.newport
        self.baudrate = self.newbaudrate
        self.bytesize = self.newbytesize
        self.stopbits = self.newstopbits
        self.timeout = self.newtimeout
        self.parity = self.newparity
        # 保险地写一个开启串口的命令
        while self.is_open is False:
            print("打开串口")
            self.open()
        if self.is_open:
            print("Successfully Connected\r\n")

    # 关闭串口
    def Serial_Close(self):
        while self.is_open is True:
            print("关闭串口")
            self.close()


# 初始化并返回一个雷达实例
def LidarInit():
    # 初始化雷达实例
    Lidar = LidarControl(port=lidar_com_name,
                         baudrate=lidar_baudrate,
                         bytesize=8,
                         stopbits=1,
                         timeout=0.6,
                         parity='N')
    # 寻找并完成串口初始化（不断循环）
    Lidar.LidarSerial_Connect()
    Lidar.LidarStart()
    return Lidar


def MpuComInit():
    Mpu1 = ComCreator(port=mpu_com_name,
                      baudrate=mpu_baudrate,
                      bytesize=8,
                      stopbits=1,
                      timeout=0.6,
                      parity='N')
    Mpu1.Serial_Connect()
    return Mpu1


# 利用点云数据绘制小短线，用以尝试构造连贯的圆弧。
def DrawStub():
    pass


# 利用点云数据绘散点圆
def DrawPoint(location, img_black):
    # 极坐标转直角坐标(注意雷达的极坐标系与我们平时用的不同，x与y轴的求法恰好相反)
    Polar_angle, Polar_diameter = location
    Polar_angle = Polar_angle * math.pi / 180  # 角度转化为弧度
    y, x = Polar_diameter * np.cos(Polar_angle), Polar_diameter * np.sin(Polar_angle)
    # 转换为OpenCV坐标系（x坐标右移，y不变）
    x, y = int(x + img_xsize / 2), int(y)
    # 绘图（绘制出来的图相当于二值化的图）
    cv2.circle(img_black, (x, y), point_r, (255, 255, 255), -1)  # 画白色实心圆点


# 将不连贯的区域连贯化（实际未用到此函数）
def LinkUpBreakpoint(point_img):
    cv2.dilate(point_img, (5, 5))  # 膨胀操作以求实现连贯。


# 骨架提取(先二值化，然后膨胀以求实现图像连贯）
def Skeleton(img_sk):
    # 二值化
    _, binary = cv2.threshold(img_sk, 200, 255, cv2.THRESH_BINARY)
    # 连贯化
    cv2.dilate(binary, (5, 5), iterations=3)  # 膨胀操作以求实现连贯。
    # 格式转换
    binary[binary == 255] = 1
    # 骨架提取
    skeleton_in = morphology.skeletonize(binary)
    skeleton_in = skeleton_in.astype(np.uint8) * 255  # 转换为一般图像格式
    # 多次膨胀操作适当放大骨架。
    skeleton_in = cv2.morphologyEx(skeleton_in, cv2.MORPH_DILATE, (5, 5), iterations=10)
    # 滤波平滑边缘
    skeleton_in = cv2.erode(skeleton_in, (3, 3))
    skeleton_in = cv2.medianBlur(skeleton_in, 7)
    # 骨架提取效果展示
    # StdShowImage(skeleton_in, "DilateSkeleton")
    return skeleton_in


# 返回图中圆的坐标值及半径、圆的数量（用于单片机判断到底要取哪个位置的圆盘，同时方便其编程）
def FindCircle(img_bi):  # 二值化图和灰度图本质上是一致的，他们都是单通道的，除了取值范围差异，其它没有。二值化图可看做灰度图的一种。
    # 图像大小要固定住！（霍夫变换受图像大小影响，为此，要固定下来图像大小，或者）
    # 霍夫圈变换（输入单通道图像）
    circles = cv2.HoughCircles(img_bi, cv2.HOUGH_GRADIENT, dp=DP, minDist=MinDist, param1=Param1, param2=Param2,
                               minRadius=MinRadius, maxRadius=MaxRadius)
    # circles是一个二维列表，第一维是第几个圆，第二维又有三层，分别是x，y，r
    if circles is None:
        return None, None  # 无圆，返回none
    else:
        return circles, circles.shape[1]  # 返回圆列表以及一共识别到几个圆（至少一个）。
    # 注意，此时返回的圆的坐标是图像坐标，对于x坐标的实际值还需要减去img_ysize。


# 绘制拟合圆（调试观察使用）
def DrawFitCircles(circles, cimg):
    circles = np.uint16(circles)  # 转换类型
    cimg = cv2.cvtColor(cimg, cv2.COLOR_GRAY2BGR)
    for i in circles[0, :]:  # 圆的第一维要固定为0，第二维才是所需的圆
        # 绘制外圆
        cv2.circle(cimg, (i[0], i[1]), i[2], (0, 255, 0), 10)
        # 绘制圆心
        cv2.circle(cimg, (i[0], i[1]), 2, (0, 0, 255), 10)
    return cimg


# 不同坐标系角度偏差修正（输入rad，输出deg）
def AngleCorrect_Rad2Deg(rad_angle, dist):
    if rad_angle > math.pi / 2:
        rad_angle = rad_angle - math.pi / 2
        angle = math.atan((dist * math.sin(rad_angle) + Lx) / (Ly + dist * abs(math.cos(rad_angle))))
        angle = angle + math.pi / 2
        return int(angle * 180 / math.pi)
    else:
        angle = math.atan((dist * math.sin(rad_angle) + Lx) / (Ly + dist * abs(math.cos(rad_angle))))
        return int(angle * 180 / math.pi)


# 找到最近的圆（极径最小的圆，该函数编写时要格外注意数据格式）
def NearestCircle(circless):
    mindist = 3000
    i = -1
    order = 0
    for circle in circless[0, :]:
        i = i + 1
        if ((circle[0] - img_xsize / 2) ** 2 + circle[1] ** 2) ** 0.5 < mindist:
            mindist = ((circle[0] - img_xsize / 2) ** 2 + circle[1] ** 2) ** 0.5
            order = i
    return circless[0, order]


def StdShowImage(image, WindowName):  # 图像窗口可以放缩的,因此就不太需要对原图进行放缩处理了
    cv2.namedWindow(WindowName, cv2.WINDOW_NORMAL)
    img_b = image.copy()
    # img_b = cv2.resize(img_b, None, fx=0.25, fy=0.25, interpolation=cv2.INTER_AREA)
    img_b = cv2.flip(img_b, 0)
    cv2.imshow(WindowName, img_b)
    cv2.waitKey(5)  # 简单等10ms，防止卡死
    # cv2.destroyWindow(WindowName)  # 关闭窗口，防止卡死


# 输入参数为实例化的雷达对象，然后便开始识别圆。
def LidarSearching_Process(lidar__ser, Mpu):  # 必须要放在进程中，不断循环进行。否则会卡死程序在这里
    i = 0
    # 初始化生成黑色背景图（事实上不需要，但为了不报错，这里写上）
    img_black = np.zeros((img_ysize, img_xsize), np.uint8)  # x坐标
    while True:
        angle, dis = lidar__ser.LidarMeasure()  # 在循环中不断调用雷达类的测距方法
        # 配合非空判断来完成循环输出正确的对应数据。
        if (angle is not None) and not (right_bound <= angle <= left_bound) and (dis is not 0):
            i = i + 1  # i用于判断是否完全绘制完一次。当i大于180，表明肯定已经绘制完成了一次了。i阈值越大，重复绘制次数越多。
            DrawPoint([angle, dis], img_black)
            if i > 400:
                # # 调试观察
                img_black = Skeleton(img_black)  # 先提取骨架
                # StdShowImage(img_black, "DrawnImage")
                print("正在识别……")
                # 识别圆的位置及大小、数量
                circles, quantity = FindCircle(img_black)  # 返回得到的数据，其中值均为整型
                if circles is not None:
                    # print(circles, quantity)
                    circle = NearestCircle(circles)  # 找到最近的圆
                    print("最近的圆为:", circle)
                    mag, ang = cv2.cartToPolar(int(circle[0] - img_xsize / 2), int(circle[1]))  # 转换为极坐标输出。
                    ang_deg = AngleCorrect_Rad2Deg(ang[0], mag[0])  # 返回角度信息（整数）
                    print("修正后角度为:", ang_deg)
                    Mpu.write("&".encode() + str(ang_deg).encode() + "e".encode())  # 向单片机串口发送角度，帧头%
                    # # ----调试用（绘制拟合圆并展示）-----
                    # print(circles, quantity)
                    img_black = DrawFitCircles(circles, img_black)  # 绘制拟合圆
                    cv2.circle(img_black, (1200, 0), 6, (0, 0, 255), 10)  # 绘制极坐标系极点
                    StdShowImage(img_black, "DetectedImage")
                    # # ---------------------------
                else:
                    print("未识别到圆")
                # 重置
                i = 0
                img_black = np.zeros((img_ysize, img_xsize), np.uint8)
                lidar__ser.LidarCom.flushInput()  # 清除所有输入缓存，提高动态性


"""主进程用以开启子进程"""
if __name__ == '__main__':
    lidar = LidarInit()
    MpuSer = MpuComInit()
    LidarSearching_Process(lidar, MpuSer)
