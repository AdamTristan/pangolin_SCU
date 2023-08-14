import cv2 as cv
import numpy as np

cap_corner1_stop = cv.VideoCapture('corner1_stop.MP4')
cap_corner234_stop = cv.VideoCapture('corner234_stop.MP4')
cap_mid_stop = cv.imread('mid_stop.PNG')
# cap_corner123_turn = cv.VideoCapture('corner123_turn.MP4')
cap_corner123_turn = cv.VideoCapture('black_corner_0414.MP4')

cap_corner4_turn = cv.VideoCapture('corner4_turn.MP4')


# TODO：你需要更改阈值
global lower_black
lower_black = np.array([0, 0, 0])
global upper_black
upper_black = np.array([255, 255, 70])

global lower_red
lower_red = np.array([0, 90, 102])
global upper_red
upper_red = np.array([186, 255, 255])

kernel = np.ones((3, 3), dtype=np.uint8)

stop_num = 0
isStop = 0  # 是否停止
isTurn = 1  # 是否转动
motion_flag = 0

k, k1, k2, b1, b2, x_turn = 0, 0, 0, 0, 0, 0


def stop_corner1():

    global stop_num
    global isStop
    global k1, k2, b1, b2, x_turn

    left_most_list = []  # 最左端坐标列表
    right_most_list = []  # 最右端坐标列表
    top_most_list = []  # 最上端坐标列表
    bottom_most_list = []  # 最下端坐标列表

    ret, image_corner1_stop = cap_corner1_stop.read()
    img = image_corner1_stop

    width = 800
    height = 600

    img = cv.resize(img, (width, height), interpolation=cv.INTER_CUBIC)

    half_height = height / 2
    half_width = width / 2
    right_line_most_left_point = width  # 初始右端线的最左边点
    left_line_most_right_point = 0  # 初始左端线的最右边点
    top_line_most_top_point = height  # 初始上端线的最上边点
    img_hsv = cv.cvtColor(img, cv.COLOR_BGR2HSV)  # BGR to HSV
    mask_red_1 = cv.inRange(img_hsv, lower_red, upper_red)

    cv.imshow('floor_mask', mask_red_1)
    cv.imshow('floor_frame', img)

    # TODO:应由你决定是否增加腐蚀操作来减少噪声（其中，mask_red_3增加了腐蚀操作，mask_red_2增加了高斯模糊操作）
    # mask_red_2 = cv.GaussianBlur(mask_red_1, (3, 3), 0)
    # contours, hierarchy = cv.findContours(mask_red_2, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_NONE)
    # mask_red_3 = cv.erode(mask_red_2, kernel, iterations=2)
    # contours, hierarchy = cv.findContours(mask_red_3, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_NONE)

    contours, hierarchy = cv.findContours(mask_red_1, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_NONE)

    for cnt in contours:
        (x, y, w, h) = cv.boundingRect(
            cnt)  # xy are the coordinate of upper left corner point，w stands for width，h stand for height
        if (w > 50 and h > 100):
            left_most = tuple(cnt[cnt[:, :, 0].argmin()][0])  # warning! The type of left_most if numpy.intc != int
            left_most_list.append(left_most)

            if (left_most[0] > half_width and left_most[0] < right_line_most_left_point):
                right_line_most_left_point = left_most[0]

            right_most = tuple(cnt[cnt[:, :, 0].argmax()][0])
            right_most_list.append(right_most)

            if (right_most[0] < half_width and right_most[0] > left_line_most_right_point):
                left_line_most_right_point = right_most[0]

            bottom_most = tuple(cnt[cnt[:, :, 1].argmax()][0])
            bottom_most_list.append(bottom_most)

            top_most = tuple(cnt[cnt[:, :, 1].argmin()][0])
            top_most_list.append(top_most)

            if (top_most[1] < top_line_most_top_point):
                top_line_most_top_point = top_most[0]

            cv.rectangle(img, (x, y), (x + w, y + h), (255, 0, 0), 1, cv.LINE_AA)

    # TODO:下述两个if语句应该在落地检测时仅保留一条，因为根据相机的不同，仅会出现一种停止姿态
    # 如果此时有两条线的轮廓
    # if (len(top_most_list) == 2):
    #     if half_height < top_most_list[0][1] and isStop == 0 and stop_num > 40:
    #         print("Stop!")
    #         isStop = 1  # 停止
    #         turnStop = 1  # 开始转向
    #     elif half_height < top_most_list[0][1] and isStop == 0:
    #         stop_num += 1
    #         print(str(stop_num))

    # TODO:stop_num以及top_most_list[0][1]的范围是需要测试更改的数值
    # 如果此时有一条线的轮廓
    if (len(top_most_list) == 1):
        if 500 < bottom_most_list[0][1] and isStop == 0 and stop_num > 40:
            print("Stop!")
            isStop = 1  # 停止
            turnStop = 1  # 开始转向
        elif 500 < bottom_most_list[0][1] and isStop == 0:
            stop_num += 1
            print(str(stop_num))

    cv.imshow('contours', img)


def stop_corner234():

    global stop_num
    global isStop
    global k1, k2, b1, b2, x_turn

    left_most_list = []  # 最左端坐标列表
    right_most_list = []  # 最右端坐标列表
    top_most_list = []  # 最上端坐标列表
    bottom_most_list = []  # 最下端坐标列表

    ret, image_corner234_stop = cap_corner234_stop.read()
    img = image_corner234_stop

    width = 800
    height = 600

    img = cv.resize(img, (width, height), interpolation=cv.INTER_CUBIC)

    half_height = height / 2
    half_width = width / 2
    right_line_most_left_point = width  # 初始右端线的最左边点
    left_line_most_right_point = 0  # 初始左端线的最右边点
    top_line_most_top_point = height  # 初始上端线的最上边点
    img_hsv = cv.cvtColor(img, cv.COLOR_BGR2HSV)  # BGR to HSV
    mask_red_1 = cv.inRange(img_hsv, lower_red, upper_red)

    cv.imshow('floor_mask', mask_red_1)
    cv.imshow('floor_frame', img)

    # TODO:应由你决定是否增加腐蚀操作来减少噪声（其中，mask_red_3增加了腐蚀操作，mask_red_2增加了高斯模糊操作）
    # mask_red_2 = cv.GaussianBlur(mask_red_1, (3, 3), 0)
    # contours, hierarchy = cv.findContours(mask_red_2, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_NONE)
    # mask_red_3 = cv.erode(mask_red_2, kernel, iterations=2)
    # contours, hierarchy = cv.findContours(mask_red_3, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_NONE)

    contours, hierarchy = cv.findContours(mask_red_1, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_NONE)

    for cnt in contours:
        (x, y, w, h) = cv.boundingRect(
            cnt)  # xy are the coordinate of upper left corner point，w stands for width，h stand for height
        if (w < 200 and h > 100):  # TODO:此处y的范围代表我不看这个像素范围以外的噪声轮廓，你应该看情况改变
            left_most = tuple(cnt[cnt[:, :, 0].argmin()][0])  # warning! The type of left_most if numpy.intc != int
            left_most_list.append(left_most)

            if (left_most[0] > half_width and left_most[0] < right_line_most_left_point):
                right_line_most_left_point = left_most[0]

            right_most = tuple(cnt[cnt[:, :, 0].argmax()][0])
            right_most_list.append(right_most)

            if (right_most[0] < half_width and right_most[0] > left_line_most_right_point):
                left_line_most_right_point = right_most[0]

            bottom_most = tuple(cnt[cnt[:, :, 1].argmax()][0])
            bottom_most_list.append(bottom_most)

            top_most = tuple(cnt[cnt[:, :, 1].argmin()][0])
            top_most_list.append(top_most)

            if (top_most[1] < top_line_most_top_point):
                top_line_most_top_point = top_most[0]

            cv.rectangle(img, (x, y), (x + w, y + h), (255, 0, 0), 1, cv.LINE_AA)

    # TODO:stop_num以及top_most_list[0][1]的范围是需要测试更改的数值
    # 如果此时有一条线的轮廓
    if (len(top_most_list) == 1):
        if half_height < top_most_list[0][1] and isStop == 0 and stop_num > 40:
            print("Stop!")
            isStop = 1  # 停止
            turnStop = 1  # 开始转向
        elif half_height < top_most_list[0][1] and isStop == 0:
            stop_num += 1
            print(str(stop_num))

    cv.imshow('contours', img)


def stop_mid():

    global stop_num
    global isStop
    global k1, k2, b1, b2, x_turn

    left_most_list = []  # 最左端坐标列表
    right_most_list = []  # 最右端坐标列表
    top_most_list = []  # 最上端坐标列表
    bottom_most_list = []  # 最下端坐标列表

    # ret, image_mid_stop = cap_mid_stop.read()
    # img = image_mid_stop
    img = cap_mid_stop
    width = 800
    height = 600

    img = cv.resize(img, (width, height), interpolation=cv.INTER_CUBIC)

    half_height = height / 2
    half_width = width / 2
    right_line_most_left_point = width  # 初始右端线的最左边点
    left_line_most_right_point = 0  # 初始左端线的最右边点
    top_line_most_top_point = height  # 初始上端线的最上边点
    img_hsv = cv.cvtColor(img, cv.COLOR_BGR2HSV)  # BGR to HSV
    mask_red_1 = cv.inRange(img_hsv, lower_red, upper_red)

    cv.imshow('floor_mask', mask_red_1)
    cv.imshow('floor_frame', img)

    # TODO:应由你决定是否增加腐蚀操作来减少噪声（其中，mask_red_3增加了腐蚀操作，mask_red_2增加了高斯模糊操作）
    # mask_red_2 = cv.GaussianBlur(mask_red_1, (3, 3), 0)
    # contours, hierarchy = cv.findContours(mask_red_2, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_NONE)
    # mask_red_3 = cv.erode(mask_red_2, kernel, iterations=2)
    # contours, hierarchy = cv.findContours(mask_red_3, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_NONE)

    contours, hierarchy = cv.findContours(mask_red_1, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_NONE)

    for cnt in contours:
        (x, y, w, h) = cv.boundingRect(
            cnt)  # xy are the coordinate of upper left corner point，w stands for width，h stand for height
        if (w > 50 and h > 50 and 200 < y < 400):  # TODO:此处y的范围代表我不看这个像素范围以外的噪声轮廓，你应该看情况改变
            left_most = tuple(cnt[cnt[:, :, 0].argmin()][0])  # warning! The type of left_most if numpy.intc != int
            left_most_list.append(left_most)

            if (left_most[0] > half_width and left_most[0] < right_line_most_left_point):
                right_line_most_left_point = left_most[0]

            right_most = tuple(cnt[cnt[:, :, 0].argmax()][0])
            right_most_list.append(right_most)

            if (right_most[0] < half_width and right_most[0] > left_line_most_right_point):
                left_line_most_right_point = right_most[0]

            bottom_most = tuple(cnt[cnt[:, :, 1].argmax()][0])
            bottom_most_list.append(bottom_most)

            top_most = tuple(cnt[cnt[:, :, 1].argmin()][0])
            top_most_list.append(top_most)

            if (top_most[1] < top_line_most_top_point):
                top_line_most_top_point = top_most[0]

            cv.rectangle(img, (x, y), (x + w, y + h), (255, 0, 0), 1, cv.LINE_AA)

    # TODO:stop_num以及top_most_list[0][1]的范围是需要测试更改的数值
    # 如果此时有一条线的轮廓
    if (len(top_most_list) == 1):
        if half_height < top_most_list[0][1] and isStop == 0 and stop_num > 40:
            print("Stop!")
            isStop = 1  # 停止
            turnStop = 1  # 开始转向
        elif half_height < top_most_list[0][1] and isStop == 0:
            stop_num += 1
            print(str(stop_num))

    cv.imshow('contours', img)


def turn_corner123():
    global stop_num
    global isTurn
    global isStop
    global motion_flag
    global k

    left_most_list = []  # 最左端坐标列表
    right_most_list = []  # 最右端坐标列表
    top_most_list = []  # 最上端坐标列表
    bottom_most_list = []  # 最下端坐标列表

    ret, image_corner123_turn = cap_corner123_turn.read()
    img = image_corner123_turn

    width = 800
    height = 600

    img = cv.resize(img, (width, height), interpolation=cv.INTER_CUBIC)

    half_height = height / 2
    half_width = width / 2
    right_line_most_left_point = width  # 初始右端线的最左边点
    left_line_most_right_point = 0  # 初始左端线的最右边点
    top_line_most_top_point = height  # 初始上端线的最上边点
    img_hsv = cv.cvtColor(img, cv.COLOR_BGR2HSV)  # BGR to HSV
    mask_red_1 = cv.inRange(img_hsv, lower_red, upper_red)
    contours, hierarchy = cv.findContours(mask_red_1, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_NONE)

    # # TODO:应由你决定是否增加腐蚀操作来减少噪声（其中，mask_red_3增加了腐蚀操作，mask_red_2增加了高斯模糊操作）
    # mask_red_2 = cv.GaussianBlur(mask_red_1, (3, 3), 0)
    # contours, hierarchy = cv.findContours(mask_red_2, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_NONE)
    # mask_red_3 = cv.erode(mask_red_2, kernel, iterations=2)
    # contours, hierarchy = cv.findContours(mask_red_3, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_NONE)

    cv.imshow('floor_mask', mask_red_1)
    cv.imshow('floor_frame', img)

    for cnt in contours:
        (x, y, w, h) = cv.boundingRect(cnt)  # xy are the coordinate of upper left corner point，w stands for width，h stand for height
        if w > 50 and h > 50:  # TODO:此处y的范围代表我不看这个像素范围以外的噪声轮廓，你应该看情况改变，可以是加and y>200
            left_most = tuple(cnt[cnt[:, :, 0].argmin()][0])  # warning! The type of left_most if numpy.intc != int
            left_most_list.append(left_most)

            if (left_most[0] > half_width and left_most[0] < right_line_most_left_point):
                right_line_most_left_point = left_most[0]

            right_most = tuple(cnt[cnt[:, :, 0].argmax()][0])
            right_most_list.append(right_most)

            if (right_most[0] < half_width and right_most[0] > left_line_most_right_point):
                left_line_most_right_point = right_most[0]

            bottom_most = tuple(cnt[cnt[:, :, 1].argmax()][0])
            bottom_most_list.append(bottom_most)

            top_most = tuple(cnt[cnt[:, :, 1].argmin()][0])
            top_most_list.append(top_most)

            if (top_most[1] < top_line_most_top_point):
                top_line_most_top_point = top_most[0]

            cv.rectangle(img, (x, y), (x + w, y + h), (255, 0, 0), 1, cv.LINE_AA)

    if isTurn == 1 and len(top_most_list) == 1:
        if top_most_list[0][0] - left_most_list[0][0] != 0:
            k = (top_most_list[0][1] - left_most_list[0][1]) / (top_most_list[0][0] - left_most_list[0][0])

        # TODO:你需要传回 k 的值来作为航向角偏移的依据（此处为转角调整）
        if 0 <= abs(k) <= 0.12:
            print("go straight!")
        else:
            print("k:" + str(k))

    cv.imshow('contours', img)


def turn_corner4():
    global stop_num
    global isTurn
    global isStop
    global motion_flag
    global k1, k2, b1, b2, x_turn

    left_most_list = []  # 最左端坐标列表
    right_most_list = []  # 最右端坐标列表
    top_most_list = []  # 最上端坐标列表
    bottom_most_list = []  # 最下端坐标列表

    ret, image_corner4_turn = cap_corner4_turn.read()
    img = image_corner4_turn

    width = 800
    height = 600

    img = cv.resize(img, (width, height), interpolation=cv.INTER_CUBIC)

    half_height = height / 2
    half_width = width / 2
    right_line_most_left_point = width  # 初始右端线的最左边点
    left_line_most_right_point = 0  # 初始左端线的最右边点
    top_line_most_top_point = height  # 初始上端线的最上边点
    img_hsv = cv.cvtColor(img, cv.COLOR_BGR2HSV)  # BGR to HSV
    mask_red_1 = cv.inRange(img_hsv, lower_red, upper_red)
    contours, hierarchy = cv.findContours(mask_red_1, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_NONE)

    # TODO:应由你决定是否增加腐蚀操作来减少噪声（其中，mask_red_3增加了腐蚀操作，mask_red_2增加了高斯模糊操作）
    # mask_red_2 = cv.GaussianBlur(mask_red_1, (3, 3), 0)
    # contours, hierarchy = cv.findContours(mask_red_2, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_NONE)
    # mask_red_3 = cv.erode(mask_red_2, kernel, iterations=2)
    # contours, hierarchy = cv.findContours(mask_red_3, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_NONE)

    cv.imshow('floor_mask', mask_red_1)
    cv.imshow('floor_frame', img)

    for cnt in contours:
        (x, y, w, h) = cv.boundingRect(
            cnt)  # xy are the coordinate of upper left corner point，w stands for width，h stand for height
        if w > 50 and h > 50 and y > 300:  # TODO:此处y的范围代表我不看这个像素范围以外的噪声轮廓，你应该看情况改变.
            left_most = tuple(cnt[cnt[:, :, 0].argmin()][0])  # warning! The type of left_most if numpy.intc != int
            left_most_list.append(left_most)

            if (left_most[0] > half_width and left_most[0] < right_line_most_left_point):
                right_line_most_left_point = left_most[0]

            right_most = tuple(cnt[cnt[:, :, 0].argmax()][0])
            right_most_list.append(right_most)

            if (right_most[0] < half_width and right_most[0] > left_line_most_right_point):
                left_line_most_right_point = right_most[0]

            bottom_most = tuple(cnt[cnt[:, :, 1].argmax()][0])
            bottom_most_list.append(bottom_most)

            top_most = tuple(cnt[cnt[:, :, 1].argmin()][0])
            top_most_list.append(top_most)

            if (top_most[1] < top_line_most_top_point):
                top_line_most_top_point = top_most[0]

            cv.rectangle(img, (x, y), (x + w, y + h), (255, 0, 0), 1, cv.LINE_AA)

    if isTurn == 1 and len(top_most_list) == 1:
        if top_most_list[0][0] - left_most_list[0][0] != 0:
            k1 = (top_most_list[0][1] - left_most_list[0][1]) / (top_most_list[0][0] - left_most_list[0][0])
        if top_most_list[0][0] - right_most_list[0][0] != 0:
            k2 = (top_most_list[0][1] - right_most_list[0][1]) / (top_most_list[0][0] - right_most_list[0][0])

        # TODO:你需要传回x_turn的值来作为航向角偏移的依据（此处为转角调整）
        if k1 != k2:
            x_turn = top_most_list[0][0]
            y = top_most_list[0][1]
            if x_turn >= 0:
                cv.circle(img, (x_turn, y), 5, (0, 255, 0), -1, cv.LINE_AA)
            sum_k = k1 + k2
            print("x_turn:" + str(x_turn))
            print("sum_k:" + str(sum_k))
        else:
            print("go straight!")

    cv.imshow('contours', img)


if __name__ == '__main__':
    # while (cap_corner1_stop.isOpened()):
    #     stop_corner1()
    #     if cv.waitKey(1) & 0xFF == ord('q'):
    #         break

    # while (cap_corner234_stop.isOpened()):
    #     stop_corner234()
    #     if cv.waitKey(1) & 0xFF == ord('q'):
    #         break

    # # TODO:请改成 while (cap_corner234_stop.isOpened()):
    # while (1):
    #     stop_mid()
    #     if cv.waitKey(1) & 0xFF == ord('q'):
    #         break

    # while (cap_corner123_turn.isOpened()):
    #     turn_corner123()
    #     if cv.waitKey(1) & 0xFF == ord('q'):
    #         break

    while (cap_corner4_turn.isOpened()):
        turn_corner4()
        if cv.waitKey(1) & 0xFF == ord('q'):
            break

    cap_corner1_stop.release()
    cv.destroyAllWindows()