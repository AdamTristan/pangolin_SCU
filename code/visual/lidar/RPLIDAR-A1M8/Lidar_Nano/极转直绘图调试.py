import cv2
import numpy as np
import math

img_xsize = 500
img_ysize = 500
img_b = np.zeros((img_ysize, img_xsize), np.uint8)
point_r = 3


# 利用点云数据绘圆
def DrawCircle(location, img_black):
    # 极坐标转直角坐标(注意雷达的极坐标系与我们平时用的不同，x与y轴的求法恰好相反)
    Polar_angle, Polar_diameter = location
    print("绘点:", Polar_angle, Polar_diameter)
    y, x = Polar_diameter * np.cos(Polar_angle), Polar_diameter * np.sin(Polar_angle)
    # 转换为OpenCV坐标系（x坐标右移）
    x, y = int(x + img_xsize/2), int(y + img_ysize/2)
    # 绘图（绘制出来的图相当于二值化的图）
    cv2.circle(img_black, (x, y), point_r, (255, 255, 255), -1)


if __name__ == '__main__':
    for i in range(0, 270):
        DrawCircle((i*math.pi/180, 100), img_b)
    cv2.imshow("out", img_b)
    cv2.waitKey(0)
