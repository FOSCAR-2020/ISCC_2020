#!/usr/bin/env python
import cv2
import threading
# import Queue as que
import time
import numpy as np

# import roslib
import sys
# import rospy

import importlib
# import cPickle
# import genpy.message
# from rospy import ROSException
# import sensor_msgs.msg
# import actionlib
# import rostopic
# import rosservice
# from rosservice import ROSServiceException

from slidewindow import SlideWindow
from warper import Warper
from pidcal import PidCal


warper = Warper()
slidewindow  = SlideWindow()
pidcal = PidCal()

cv_image = None
ack_publisher = None
car_run_speed = 0.5

def auto_drive(pid):
    global car_run_speed
    w = 0
    if -0.065 < pid and pid < 0.065:
        w = 1
    else:
        w = 0.3

    if car_run_speed < 1.0 * w:
        car_run_speed += 0.002 * 10
    else:
        car_run_speed -= 0.003 * 10

    drive_value = drive_values()
    drive_value.throttle = int(3)
    drive_value.steering = (pid/0.074)

    drive_values_pub.publish(drive_value)
    print('steer: ', drive_value.steering)
    print('speed: ', car_run_speed)

def main():

    # cap = cv2.VideoCapture(0);
    # cap = cv2.VideoCapture(1)
    cap = cv2.VideoCapture("TEST14.avi")
    # cap.set(CV_CAP_PROP_FRAME_WIDTH,800)
    # cap.set(CV_CAP_PROP_FRAME_HEIGHT,448)
    cap.set(3,800)
    cap.set(4,448)

    while True:
        ret, img = cap.read()
        img1, x_location = process_image(img)
        if x_location != None:
            pid = round(pidcal.pid_control(int(x_location)), 6)
            # auto_drive(pid)
        cv2.imshow('result', img1)
        # print (pid)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

      #out.write(img1)
      #out2.write(cv_image)

def light_calc(frame):
    arr = np.array(frame)

    max_val = np.max(arr)
    min_val = np.min(arr)

    dst = (frame - min_val)*(255/(max_val - min_val))

    # print('min_val : ' ,min_val)
    # print('max_val : ', max_val )

    # cv2.imshow("dst",dst)

    return dst

def stop_line(frame):

    out_img = np.dstack((frame, frame, frame)) * 255
    height = frame.shape[0]
    width = frame.shape[1]

	# inputImage = gray[y:y+h, x:x+w]

    x = 180
    w = 440
    y = 320
    h = 80

    x1 = 0
    y1 = 0
    x2 = 0
    y2 = 0

    cnt = 0

    roi_img = frame[y:y+h, x:x+w]

    lines = cv2.HoughLines(roi_img,1,np.pi/180,100)
    print(lines)

    if lines is None:
        return [-1, -1, -1, -1]
    else :
        for i in range(len(lines)):
            for rho, theta in lines[i]:
                tempTheta = theta/np.pi *180
                if(tempTheta > 88 and tempTheta < 92):
                    cnt = cnt + 1

                    a = np.cos(theta)
                    b = np.sin(theta)
                    x0 = a*rho
                    y0 = b*rho
                    x1 += int(x0 + 1000*(-b))
                    y1 += int(y0 + 1000*(a))
                    x2 += int(x0 - 1000*(-b))
                    y2 += int(y0 -1000*(a))
        if cnt != 0:
            return [int(x1/cnt), int(y1/cnt + y), int(x2/cnt), int(y2/cnt + y)]
        else :
            return [-1, -1, -1, -1]




def process_image(frame):

    # blur
    kernel_size = 3
    blur = cv2.GaussianBlur(frame,(kernel_size, kernel_size), 1)

    blur = cv2.GaussianBlur(blur,(kernel_size, kernel_size), 1)
    blur = cv2.GaussianBlur(blur,(kernel_size, kernel_size), 1)
    blur = cv2.GaussianBlur(blur,(kernel_size, kernel_size), 1)
    blur = cv2.GaussianBlur(blur,(kernel_size, kernel_size), 1)
    blur = cv2.GaussianBlur(blur,(kernel_size, kernel_size), 1)
    blur = cv2.GaussianBlur(blur,(kernel_size, kernel_size), 1)
    blur = cv2.GaussianBlur(blur,(kernel_size, kernel_size), 1)
    blur = cv2.GaussianBlur(blur,(kernel_size, kernel_size), 1)
    blur = cv2.GaussianBlur(blur,(kernel_size, kernel_size), 1)
    blur = cv2.GaussianBlur(blur,(kernel_size, kernel_size), 1)
    blur = cv2.GaussianBlur(blur,(kernel_size, kernel_size), 1)
    blur = cv2.GaussianBlur(blur,(kernel_size, kernel_size), 1)


    cv2.imshow("blue", blur)

    # img_bird = warper.warp(frame)

    # cv2.imshow("img_bird",img_bird)

    hsv = cv2.cvtColor(blur,cv2.COLOR_BGR2HSV)
    h,s,v = cv2.split(hsv)

    hls = cv2.cvtColor(blur,cv2.COLOR_BGR2HLS)
    h,l,s = cv2.split(hls)

    # cv2.imshow("h",h)
    yellow_process_img = light_calc(s)
    white_process_img = light_calc(l)

    # cv2.imshow("enhance", yellow_process_img)
    # cv2.imshow("white_mask",white_process_img)

    # grayscle
    # gray = cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)


    # # blur
    # kernel_size = 5
    # blur_gray = cv2.GaussianBlur(gray,(kernel_size, kernel_size), 0)
    # blur_gray1 = cv2.GaussianBlur(yellow_process_img,(kernel_size, kernel_size), 0)
    # blur_gray2 = cv2.GaussianBlur(white_process_img,(kernel_size, kernel_size), 0)

    ret,binary_img = cv2.threshold(yellow_process_img, 70, 255, cv2.THRESH_BINARY)

    # cv2.imshow("bi",binary_img)

    ret1,binary_img1 = cv2.threshold(white_process_img, 150, 255, cv2.THRESH_BINARY)

    # cv2.imshow("bi1",binary_img1)

    img_mask = cv2.bitwise_or(binary_img,binary_img1)

    # img_result = cv2.bitwise_and(binary_img,binary_img,mask = img_mask)

    cv2.imshow("img_result",img_mask)




    # canny edge
    low_threshold = 60#60
    high_threshold = 70# 70

    # edges_img = cv2.Canny(np.uint8(blur_gray), low_threshold, high_threshold)
    # edges_img1 = cv2.Canny(np.uint8(binary_img), low_threshold, high_threshold)
    # edges_img2 = cv2.Canny(np.uint8(binary_img1), low_threshold, high_threshold)
    edges_img3 = cv2.Canny(np.uint8(blur), low_threshold, high_threshold)


    # cv2.imshow("edges_img1",edges_img3)

    # warper
    # img = warper.warp(edges_img)
    # bird = warper.warp(edges_img1)
    # bird1 = warper.warp(edges_img2)
    bird2 = warper.warp(edges_img3)

    # stop_line
    # 미검출시[-1, -1, -1, -1] 검출시[x1, y1, x2, y2]
    stop_line_array = stop_line(bird2)
    # print(stop_line_array)

    # cv2.imshow("bird",bird2)

    # img1, x_location1 = slidewindow.slidewindow(img)
    # img2, x_location2 = slidewindow.slidewindow(bird)
    img3, x_location3 = slidewindow.slidewindow(bird2)

    # 정지선 인식 범위
    cv2.rectangle(img3, (180, 320),  (180+440, 320+80), (0, 0, 255), 4)

    # 정지선 그리기
    if(stop_line_array[0] != -1):

        font = cv2.FONT_HERSHEY_COMPLEX  # normal size serif font
        fontScale = 1.2
        cv2.putText(img3, 'stop!!!!!!', (10, 80), font, fontScale, (0, 0, 255), 4)

        cv2.line(img3,(stop_line_array[0], stop_line_array[1]) \
        ,(stop_line_array[2], stop_line_array[3]),(0,0,255),10)



    # print(x_location1)

    return img3, x_location3

if __name__ == '__main__':
    main()
