# -*- coding:utf-8 -*-

import cv2
import numpy as np

"""
¹¦ÄÜ£º¶ÁÈ¡Ò»ÕÅÍ¼Æ¬£¬ÏÔÊ¾³öÀ´£¬×ª»¯ÎªHSVÉ«²Ê¿Õ¼ä
     ²¢Í¨¹ý»¬¿éµ÷½ÚHSVãÐÖµ£¬ÊµÊ±ÏÔÊ¾
"""

img = cv2.imread('1.PNG')
frame = cv2.resize(img, (500, 500), interpolation=cv2.INTER_AREA)

hsv_low = np.array([0, 0, 0])
hsv_high = np.array([0, 0, 0])

# ÏÂÃæ¼¸¸öº¯Êý£¬Ð´µÃÓÐµãÈßÓà

def h_low(value):
    hsv_low[0] = value

def h_high(value):
    hsv_high[0] = value

def s_low(value):
    hsv_low[1] = value

def s_high(value):
    hsv_high[1] = value

def v_low(value):
    hsv_low[2] = value

def v_high(value):
    hsv_high[2] = value

cv2.namedWindow('image',cv2.WINDOW_AUTOSIZE)
# ¿ÉÒÔ×Ô¼ºÉè¶¨³õÊ¼Öµ£¬×î´óÖµ255²»ÐèÒªµ÷½Ú
cv2.createTrackbar('H low', 'image', 0, 255, h_low)
cv2.createTrackbar('H high', 'image',255, 255, h_high)
cv2.createTrackbar('S low', 'image', 0, 255, s_low)
cv2.createTrackbar('S high', 'image', 255, 255, s_high)
cv2.createTrackbar('V low', 'image', 0, 255, v_low)
cv2.createTrackbar('V high', 'image', 180, 255, v_high)

while True:
    dst = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV) # BGR×ªHSV
    dst = cv2.inRange(dst, hsv_low, hsv_high) # Í¨¹ýHSVµÄ¸ßµÍãÐÖµ£¬ÌáÈ¡Í¼Ïñ²¿·ÖÇøÓò
    cv2.imshow('dst', dst)
    cv2.imshow('frame', frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cv2.destroyAllWindows()