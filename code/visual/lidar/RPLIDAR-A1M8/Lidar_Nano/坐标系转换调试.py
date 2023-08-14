import cv2
import math

L = 1
dist1 = 500
ang1 = 180*math.pi/180


def AngleCorrect(deg_angle, dist):
    if deg_angle > 90:
        deg_angle = deg_angle - 90
        rad_angle = deg_angle * math.pi / 180
        angle = math.atan((dist/(L+dist*abs(math.cos(rad_angle))))*math.sin(rad_angle))
        return angle + 90
    else:
        rad_angle = deg_angle * math.pi / 180
        angle = math.atan((dist / (L + dist * abs(math.cos(rad_angle)))) * math.sin(rad_angle))
        return angle


def AngleCorrect_Rad(rad_angle, dist):
    if rad_angle > math.pi/2:
        rad_angle = rad_angle - math.pi/2
        angle = math.atan((dist/(L+dist*abs(math.cos(rad_angle))))*math.sin(rad_angle))
        angle = angle + math.pi/2
        return angle*180/math.pi
    else:
        angle = math.atan((dist / (L + dist * abs(math.cos(rad_angle)))) * math.sin(rad_angle))
        return angle*180/math.pi


x = -3
y = 4
mag, ang = cv2.cartToPolar(x, y)

print("极径:", float(mag[0]), "极角:", float(ang[0]*180/math.pi))
gx, gy = cv2.polarToCart(mag, ang)
print("x=", float(gx[0]), "y=", float(gy[0]))


angel = AngleCorrect_Rad(ang[0], dist1)
print(angel)
