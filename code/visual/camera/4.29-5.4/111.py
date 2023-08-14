import cv2 as cv
import numpy as np
from matplotlib import pyplot as plt

def getTransferFun(kernel, r):  # 计算滤波器核的传递函数
    hPad, wPad = r-kernel.shape[0]//2, r-kernel.shape[1]//2
    kernPadded = cv.copyMakeBorder(kernel, hPad, hPad, wPad, wPad, cv.BORDER_CONSTANT)
    kernFFT = np.fft.fft2(kernPadded)
    fftShift = np.fft.fftshift(kernFFT)
    print(fftShift)
    kernTrans = np.log(1 + np.abs(fftShift))
    print(kernTrans)
    transNorm = np.uint8(cv.normalize(kernTrans, None, 0, 255, cv.NORM_MINMAX))
    print(transNorm)
    return transNorm

if __name__ == '__main__':
    radius = 256
    plt.figure(figsize=(9, 5.5))

    # (1) 盒式滤波器
    plt.subplot(241), plt.axis('off'), plt.title("1. BoxFilter")
    kernBox = h = np.array([[0, 1/4, 0],
              [1/4, 0, 1/4],
              [0, 1/4, 0]])  # BoxF 滤波器核
    HBox = getTransferFun(kernBox, radius)  # BoxF 传递函数
    plt.imshow(HBox, cmap='gray', vmin=0, vmax=255)

    plt.tight_layout()
    plt.show()
