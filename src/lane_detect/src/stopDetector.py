#-*- coding:utf-8 -*-
import cv2
import numpy as np

import cv2
import numpy as np
import math

#245, 248 -- 130~140

class StopLine:
    # def __init__(self):
    #     #self.cap = cv2.VideoCapture(video)

    def findline(self, img, locate_x, locate_y, length, find_l, find_r):
        stopflag = None #정지선인경우 : 1, 아닌경우 : None
        left = None #정지선의 왼쪽끝점 좌표
        right = None #정지선의 오른쪽 끝점 좌표
        stop_Length = 0 #정지선의 길이

        outimg = np.dstack((img, img, img))
        window_h = 5
        window_w = 5

        L_loca_x = locate_x
        R_loca_x = locate_x
        minpix = 23

        nonzero = img.nonzero()
        non_y = np.array(nonzero[0])
        non_x = np.array(nonzero[1])
        good_L_lst = ((non_x <= L_loca_x) & (non_x >= L_loca_x - window_w) & (non_y <= locate_y + window_h) & (non_y >= locate_y - window_h)).nonzero()[0]
        good_R_lst = ((non_x >= R_loca_x) & (non_x <= R_loca_x + window_w) & (non_y <= locate_y + window_h) & (non_y >= locate_y - window_h)).nonzero()[0]

        lcurrent = np.mean(non_x[good_L_lst])
        lycurrent = np.mean(non_y[good_L_lst])
        rcurrent = np.mean(non_x[good_R_lst])
        rycurrent = np.mean(non_y[good_R_lst])

        while (len(good_L_lst) >= minpix or len(good_R_lst) >= minpix):
            if (lcurrent < find_l or rcurrent > find_r): break #차선을 벗어나는 경우 break
            if (len(good_L_lst) >= minpix):
                good_L_lst = ((non_x <= lcurrent) & (non_x >= lcurrent - window_w) & (non_y <= lycurrent + window_h) & (non_y >= lycurrent - window_h)).nonzero()[0]
                lcurrent = np.mean(non_x[good_L_lst])
                lycurrent = np.mean(non_y[good_L_lst])
            if (len(good_R_lst) >= minpix):
                good_R_lst = ((non_x >= rcurrent) & (non_x <= rcurrent + window_w) & (non_y <= rycurrent + window_h) & (non_y >= rycurrent - window_h)).nonzero()[0]
                rcurrent = np.mean(non_x[good_R_lst])
                rycurrent = np.mean(non_y[good_R_lst])
            # cv2.circle(outimg, (int(lcurrent), locate_y), 3, (0, 150, 150), 2)
            # cv2.circle(outimg, (int(rcurrent), locate_y), 3, (150, 150, 0), 2)
        lcurrent = np.mean(non_x[good_L_lst])
        rcurrent = np.mean(non_x[good_R_lst])
        lycurrent = np.mean(non_y[good_L_lst])
        rycurrent = np.mean(non_y[good_R_lst])
        stop_Length = int(math.sqrt(pow(rcurrent - lcurrent, 2) + pow(rycurrent - lycurrent, 2)))

        print("L : ", lcurrent, " R : ", rcurrent, " LENGTH : ", stop_Length)
        if length:
            print(length)
            if (length * 0.75 < stop_Length < length * 1.25):
                stopflag = 1
                left = (int(lcurrent), int(lycurrent))
                right = (int(rcurrent), int(rycurrent))
        cv2.imshow('out', outimg)
        cv2.waitKey(0)
        return stopflag, left, right

    def processing(self):
        frame = 0
        min_frame = 7 #앞의 정지선을 발견후 min_frame수만큼의 frame은 무시
        while self.cap.isOpened():
            # threshold_value = 145
            # value = 255

            ret, img = self.cap.read()
            # src = np.float32([[60, 355],
            #                   [10, 400],
            #                   [640 - 15, 405],
            #                   [640, 360]])
            # dst = np.float32([[0, 0],
            #                   [0, 480],
            #                   [640, 480],
            #                   [640 + 50, 0]])
            # M = cv2.getPerspectiveTransform(src, dst)
            # warp = cv2.warpPerspective(gray, M, (640, 480), flags=cv2.INTER_LINEAR)
            # thres = cv2.threshold(warp, threshold_value, value, cv2.THRESH_BINARY)[1]
            if ret:
                gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
                line = h_w_linedetect.SlideWindow()
                tf, left, right, w_w_img = line.w_slidewindow(gray)
                if (tf):
                    xL, xR, outimg, theta, center, length = line.h_slidewindow(gray, left, right)
                #
                # else:
                #     continue
                    # xL, xR, outimg, theta = line.h_slidewindow(gray, old_L, old_R)
                if center != None:
                    locate_x, locate_y = center
                    if (gray[int(locate_y)][int(locate_x)] != 0):
                        stop, id_L, id_R = self.findline(gray, locate_x, locate_y, length, xL, xR)
                        if (stop != None): #정지선이 발견되었을 경우
                            if (frame == 0): #frame == 0 ~> 초기화 된 frame ~> 첫번째 정지선
                                print("STOP!")
                                cv2.line(img, id_L, id_R, (0, 0, 255), 2)
                            frame += 1
                        if (frame > min_frame): #일정한 프레임이 지나면 frame을 0으로 초기화
                            frame = 0
                        print(stop, frame)
                cv2.imshow('w_window', w_w_img)
                cv2.imshow('h_window', outimg)
                cv2.imshow('stop', img)
                if (cv2.waitKey(1) & 0xFF == 27):
                    break
