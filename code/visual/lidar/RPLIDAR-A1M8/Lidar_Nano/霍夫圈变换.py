import cv2
import numpy as np
import cv2 as cv

img = cv.imread('Test_Image/3.png', 0)
cimg = cv.cvtColor(img, cv.COLOR_GRAY2BGR)
# 霍夫圈变换。
circles = cv.HoughCircles(img, cv.HOUGH_GRADIENT, 1, minDist=100, param1=50, param2=30, minRadius=0, maxRadius=0)
if circles is None:
    print('未识别到圆')
    exit(0)
# circles = np.uint16(np.around(circles))  # 转换为无符号16位整型，去除小数。
print(circles.shape[1])
for i in circles[0, :]:
    # # 绘制外圆
    # cv.circle(cimg, (i[0], i[1]), i[2], (0, 255, 0), 2)
    # # 绘制圆心
    # cv.circle(cimg, (i[0], i[1]), 2, (0, 0, 255), 3)
    print(i[0], i[1], i[2], "\r\n")
# 可见，绘制圆是在原图上进行修改的！这一点特性其实蛮好用的！！
cimg = cv.resize(cimg, None, fx=0.25, fy=0.25, interpolation=cv.INTER_AREA)  # 缩小显示，防止图像过大。
cv.imshow('detected circles', cimg)
cv.waitKey(0)
cv.destroyAllWindows()

"""
对于图3和图4，只有图像大小的差别。图3在param2 = 30时可以检测的那个最小的圆弧，但是图4就不行，原因在于图4比图3要小很多，因而
可以供圆变换用的像素点就少很多，导致变换有问题。但是，条件param2=20时，小圆也出现了！因此，图像的大小设置，参数2的配置对于圆检测
具有比较大的影响。但是，当param2=20时，我们选择图5做测试，他将把非圆的折线也一起识别为圆，可见20相对较低了。因此， 一种比较好的思路，
是增大图像的大小（尽管会增加一些计算量，但毕竟又不是深度学习），同时提高参数2的数值。
同时根据实际图像大小以及代拟合的最小圆的大小，应该合理地调整minDist大小，以避免重复识别。
另外，合理地调整minRadius=0, maxRadius=0也可以进一步避免误识别。
值得注意的是，对于啥也没有的图片7，程序会报错。因此，我们需要有容错判断。
"""