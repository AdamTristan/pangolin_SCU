import numpy as np
import cv2 as cv
from skimage import morphology


def StdShowImage(image, WindowName):
    cv.namedWindow(WindowName, cv.WINDOW_NORMAL)
    cv.imshow(WindowName, image)
    cv.waitKey(0)
    cv.destroyAllWindows()


def Skeleton(img_sk):
    #  二值化
    _, binary = cv.threshold(img_sk, 200, 255, cv.THRESH_BINARY)
    binary[binary == 255] = 1
    # 骨架提取
    skeleton_in = morphology.skeletonize(binary)
    skeleton_in = skeleton_in.astype(np.uint8) * 255
    skeleton_in = cv.morphologyEx(skeleton_in, cv.MORPH_DILATE, (5, 5), iterations=8)
    # # 膨胀操作适当放大骨架。
    # skeleton_in = cv.dilate(skeleton_in, (5, 5))
    # # 滤波平滑边缘
    # skeleton_in = cv.erode(skeleton_in, (3, 3))
    # skeleton_in = cv.medianBlur(skeleton_in, 7)
    StdShowImage(skeleton_in, "DilateSkeleton")
    return skeleton_in


# ——————————主函数—————————————————
img = cv.imread('Test_Image/12.png', 0)
skeleton = Skeleton(img.copy())  # 输入灰度图

# ————————霍夫圈变换（其本身并不绘制圆）————————
circles = cv.HoughCircles(skeleton, cv.HOUGH_GRADIENT, 1, minDist=100, param1=50, param2=12, minRadius=5, maxRadius=1000)
if circles is None:
    print('未识别到圆')
    exit(0)
else:
    circles = np.uint16(np.around(circles))  # 转换为无符号16位整型，去除小数。
    print(circles)
    print(len(circles[0, :]))

# ——————————循环绘制出所有圆并展示——————————
cimg = cv.cvtColor(skeleton, cv.COLOR_GRAY2BGR)
for i in circles[0, :]:
    # 绘制外圆
    cv.circle(cimg, (i[0], i[1]), i[2], (0, 255, 0), 2)
    # 绘制圆心
    cv.circle(cimg, (i[0], i[1]), 2, (0, 0, 255), 3)
    print(i[0], i[1], i[2], "\r\n")
cv.imshow('detected circles', cimg)
cv.waitKey(0)
cv.destroyAllWindows()

'''
对于点云绘制的圆弧，相对与标准圆弧，其更加不标准，尽管可能人眼看起来差不多，但其实相差较大，因此，往往param2较大时无法检测出正确的结果！
所以应该尽量减小该参数的值。
另外，霍夫圈变换仅能针对连贯的点进行拟合，对于离散点拟合将失效。因此，点云必须要有足够的直径以使各个点有重合部分。
'''