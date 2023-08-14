import cv2
import numpy as np
import time
img_xsize = 500
img_ysize = 500


# 测试用图像获取(输出二值图）
def Std_ImageRead():
    img = cv2.imread("Test_Image/2.png", 0)
    if img is None:
        print('Unable to read: ')
        exit(0)
    img_std = cv2.resize(img.copy(), (img_xsize, img_ysize))  # 先固定画幅大小
    _, img_bi = cv2.threshold(img_std, 70, 255, cv2.THRESH_BINARY)
    return img_bi


def Norm_ImageRead():
    img = cv2.imread("Test_Image/2.png", 0)
    if img is None:
        print('Unable to read: ')
        exit(0)
    _, img_bi = cv2.threshold(img, 70, 255, cv2.THRESH_BINARY)
    return img_bi


def Circle_Match(cnt):
    perimeter = cv2.arcLength(cnt, False)


# 返回图中圆的坐标值及半径（用于单片机判断到底要取哪个位置的圆盘）。
def FindCircle(img_bi, r_need):  # 二值化图和灰度图本质上是一致的，他们都是单通道的，除了取值范围差异，其它没有。二值化图可看做灰度图的一种。
    err = 10  # 半径波动的可接受范围
    # 霍夫圈变换（输入单通道图像）
    circles = cv2.HoughCircles(img_bi, cv2.HOUGH_GRADIENT, 1, minDist=100, param1=50, param2=30, minRadius=0, maxRadius=0)
    if circles is None:
        return None, None, None
    else:
        circles = np.uint16(np.around(circles))
        for i in circles[0, :]:
            if r_need - err < i[2] < r_need + err:
                return i[0], i[1], i[2]  # 找到了正确的圆盘，返回它的x，y坐标以及半径r
        return None, None, None


if __name__ == '__main__':
    t = time.time()
    image = Norm_ImageRead()
    cv2.imshow('1', image)
    cv2.waitKey(0) & 0xff
    cnts, hierarchy = cv2.findContours(image.copy(), cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)  # LIST模式更加稳定！
    for cnt in cnts:
        (x, y), radius = cv2.minEnclosingCircle(cnt)
        x, y, radius = int(x), int(y), int(radius)
        image = cv2.circle(image, (x, y), radius, (255, 255, 255), 3)
    cv2.imshow('2', image)
    cv2.waitKey(0) & 0xff
