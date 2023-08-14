import serial  # python串口库
import serial.tools.list_ports
import time

# 串口相关
lidar_com_name = "COM7"
# Linux   : "/dev/ttyUSB0"
# MacOS   : "/dev/cu.SLAB_USBtoUART"
# Windows : "COM7"
lidar_baudrate = 115200


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

    def LidarSerial_Close(self):
        self.LidarCom.close()

    def LidarStart(self):
        """开启雷达"""
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
                print("扫描开始")
                break
            else:
                print("重新尝试发送开启扫描命令")

    def LidarStop(self):
        """关闭激光"""
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


def LidarInit():
    # 初始化雷达实例
    Lidar = LidarControl(port=lidar_com_name,
                         baudrate=lidar_baudrate,
                         bytesize=8,
                         stopbits=1,
                         timeout=0.6,
                         parity='N')
    # 寻找并完成串口初始化
    Lidar.LidarSerial_Connect()
    Lidar.LidarStart()
    return Lidar


def LidarProcess(Lidar):
    while True:
        timeNow = time.time()
        angle, distance = Lidar.LidarMeasure()
        if angle is not None:
            deltaTime = time.time() - timeNow
            print("角度:", angle, "°", "距离:", distance, "mm", "时间：", deltaTime)



if __name__ == '__main__':
    lidar = LidarInit()
    LidarProcess(lidar)
    time.sleep(10)
    lidar.LidarStop()


