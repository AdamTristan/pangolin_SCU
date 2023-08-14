import cv2
import numpy as np
import serial  # python串口库
import serial.tools.list_ports
import time
import math

# 点云绘图的参数`
point_r = 25  # 所绘制点的半径大小（重要参数）
img_size = 3000  # 一种可能更快的方式是不创建图，而是导入图。不清楚创建大背景图是不是很快。
# 串口相关参数
lidar_com_name = "COM7"
# Linux   : "/dev/ttyUSB0"
# MacOS   : "/dev/cu.SLAB_USBtoUART"
# Windows : "COM7"
lidar_baudrate = 115200


# 利用点云数据绘圆
def DrawPoint(location, img_black):
    # 极坐标转直角坐标(注意雷达的极坐标系与我们平时用的不同，x与y轴的求法恰好相反)
    Polar_angle, Polar_diameter = location
    Polar_angle = Polar_angle*math.pi / 180  # 角度转化为弧度
    # print(Polar_angle, Polar_diameter)
    y, x = Polar_diameter * np.cos(Polar_angle), Polar_diameter * np.sin(Polar_angle)
    # 转换为OpenCV坐标系（x坐标右移）
    x, y = int(x+img_size/2), int(y+img_size/2)
    # 绘图（绘制出来的图相当于二值化的图）
    cv2.circle(img_black, (x, y), point_r, (255, 255, 255), -1)  # 画白色实心圆点


# 返回图中圆的坐标值及半径、圆的数量（用于单片机判断到底要取哪个位置的圆盘，同时方便其编程）
def FindCircle(img_bi):  # 二值化图和灰度图本质上是一致的，他们都是单通道的，除了取值范围差异，其它没有。二值化图可看做灰度图的一种。
    # 图像大小要固定住！（霍夫变换受图像大小影响，为此，要固定下来图像大小，或者）
    # 霍夫圈变换（输入单通道图像）
    circles = cv2.HoughCircles(img_bi, cv2.HOUGH_GRADIENT, 1, minDist=230, param1=50, param2=30, minRadius=80,
                               maxRadius=270)
    # circles是一个二维列表，第一维是第几个圆，第二维又有三层，分别是x，y，r
    if circles is None:
        return None, None  # 无圆，返回none
    else:
        circles = np.uint16(circles)  # 转换类型（不知道转换对还是不转换对）
        return circles, len(circles)  # 返回圆列表以及一共识别到几个圆（至少一个）。
    # 注意，此时返回的圆的坐标是图像坐标，对于x坐标的实际值还需要减去img_ysize。


def DrawFitCircle(circles, cimg):
    for i in circles[0, :]:
        # 绘制外圆
        cv2.circle(cimg, (i[0], i[1]), i[2], (0, 255, 0), 2)
        # 绘制圆心
        cv2.circle(cimg, (i[0], i[1]), 2, (0, 0, 255), 3)


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
                    print("try:", port[0:4])
                    # 如果找到，则开始串口配置
                    if port[0:4] == self.port:
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

    # 关闭激光扫描
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
        """由产品手册得知，读取从串口返回数据为...."""
        if self.LidarCom.inWaiting() > 0:  # 如果接受缓存区有数据
            res = self.LidarCom.read(1)  # 读取一个字节byte(并不是默认读取字符串）
            # ————————————————核心数据解析——————————————————————
            if self.peri_flag:  # 判断是否开启新的周期
                self.count = self.count + 1
                if self.count > 4:  # 周期结束
                    self.count = 0
                    self.peri_flag = False
                # if self.count == 1:  # 第二位是奇偶校验位，一般没问题，可以忽略
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


# 输入参数为实例化的雷达对象
def LidarSearching_Process(lidar__ser):  # 必须要放在进程中，不断循环进行。否则会卡死程序在这里。
    # 定义起始、结束区
    start_angle1 = 0
    start_angle2 = 10
    end_angle1 = 350
    end_angle2 = 360
    # 定义进/出区控制标志
    draw_flag = False
    # 初始化生成黑色背景图（事实上不需要，但为了不报错，这里写上）
    img_black = np.zeros((img_size, img_size), np.uint8)  # x坐标
    while True:
        angle, dis = lidar__ser.LidarMeasure()  # 在循环中不断调用雷达类的测距方法。
        location = [angle, dis]
        # 配合非空判断来完成循环输出正确的对应数据。
        if (angle is not None) and (dis != 0):
            # 起始区判断(保证每次绘图的完整性）
            if start_angle1 <= angle <= start_angle2 and not draw_flag:  # 当draw_flag为True时不再进行重置。即不重复重置。
                img_black = np.zeros((img_size, img_size), np.uint8)  # 重新生成黑色背景图
                draw_flag = True
                print("起始角度:", angle, "起始距离:", dis)
            # 两区之间、1/4象限进行绘图（只有判断完起始区才能进行如下绘图）
            if draw_flag:
                # ——————关键绘图函数——————
                DrawPoint(location, img_black)
            # 结束区判断（然后对图片进行处理）
            if end_angle1 <= angle <= end_angle2 and draw_flag:  # 必须要先经历起始区，才能触发结束区。
                draw_flag = False
                print("结束角度:", angle, "结束距离:", dis)
                # 调试观察
                img_b = cv2.resize(img_black.copy(), None, fx=0.25, fy=0.25, interpolation=cv2.INTER_AREA)
                cv2.imshow("DrawnImage", img_b)
                cv2.waitKey(0)
                cv2.destroyWindow("DrawnImage")
                # ————————开始识别圆的位置及大小——————
                circles, quantity = FindCircle(img_black)  # 返回得到的数据，其中值均为整型
                if circles is not None:
                    DrawFitCircle(circles, img_black)  # 绘制圆
                    img_black = cv2.resize(img_black, None, fx=0.25, fy=0.25, interpolation=cv2.INTER_AREA)
                    cv2.imshow("DetectedImage", img_black)
                    print(circles, quantity)
                    # 暂停
                    cv2.waitKey(0)
                    cv2.destroyWindow("DetectedImage")
                else:
                    print("未拟合出圆")


def DrawAll(lidar__ser):
    img_black = np.zeros((img_size, img_size), np.uint8)  # 重新生成黑色背景图
    while True:
        t = time.time()
        angle, dis = lidar__ser.LidarMeasure()  # 在循环中不断调用雷达类的测距方法。
        location = [angle, dis]
        # 配合非空判断来完成循环输出正确的对应数据。
        if (angle is not None) and (dis != 0):
            DrawPoint(location, img_black)
            img__b = cv2.resize(img_black.copy(), None, fx=0.25, fy=0.25, interpolation=cv2.INTER_AREA)
            img__b = cv2.flip(img__b, 0)
            cv2.imshow("DrawnImage", img__b)
            cv2.waitKey(1)  # 必须加一句这个，否则图像弹出框会卡死无响应
            print("角度:", angle, "距离:", dis)
        print(time.time() - t)


def TestDraw(img__b):
    for i in range(135, 360):
        DrawPoint((i, 1000), img__b)
    for i in range(0, 45):
        DrawPoint((i, 1000), img__b)
    img__b = cv2.resize(img__b, None, fx=0.25, fy=0.25, interpolation=cv2.INTER_AREA)
    img__b = cv2.flip(img__b, 0)
    cv2.imshow("out", img__b)
    cv2.waitKey(0)


"""主进程用以开启子进程"""
if __name__ == '__main__':
    img_b = np.zeros((img_size, img_size), np.uint8)
    lidar = LidarInit()
    DrawAll(lidar)
