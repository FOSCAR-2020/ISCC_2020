import cv2
import numpy as np
import matplotlib.pyplot as plt
from findpoint import FindPoint


class LineDetector:
    def __init__(self):

        self.frame = None
        self.leftx = None
        self.rightx = None
        # self.output = None
        self.frame = 0
        self.frame_list = []
        self.findpoint = FindPoint()

    def sliding_window(self,x_start_L,x_start_R,img):
        x_location = None
        out_img = np.dstack((img,img,img))
        height = img.shape[0]
        width = img.shape[1]

        window_height = 5
        nwindows = 30

        nonzero = img.nonzero()
        nonzeroy = np.array(nonzero[0])
        nonzerox = np.array(nonzero[1])

        margin = 20
        minpix = 10
        left_lane_inds = []
        right_lane_inds = []

        good_left_inds = \
        ((nonzerox >= x_start_L-20) & (nonzeroy >= 300)& (nonzeroy <= 400) & (nonzerox <= x_start_L+20)).nonzero()[
            0]
        good_right_inds = ((nonzerox >= x_start_R-40) & (nonzeroy >= 300)& (nonzeroy <= 400) & (
                    nonzerox <= x_start_R+20)).nonzero()[0]
        line_exist_flag = None
        y_current = None
        x_current = None
        good_center_inds = None
        p_cut = None

        # check the minpix before left start line
        # if minpix is enough on left, draw left, then draw right depends on left
        # else draw right, then draw left depends on right
        if len(good_left_inds) > minpix:
            line_flag = 1
            x_current = np.int(np.mean(nonzerox[good_left_inds]))
            y_current = np.int(np.mean(nonzeroy[good_left_inds]))
            max_y = y_current
        elif len(good_right_inds) > minpix:
            line_flag = 2
            x_current = nonzerox[good_right_inds[np.argmax(nonzeroy[good_right_inds])]]
            y_current = np.int(np.max(nonzeroy[good_right_inds]))
        if len(good_left_inds) > minpix and len(good_right_inds) > minpix:
            line_flag = 3
            lx_current = np.int(np.mean(nonzerox[good_left_inds]))
            ly_current = np.int(np.mean(nonzeroy[good_left_inds]))
            rx_current = np.int(np.mean(nonzerox[good_right_inds]))
            ry_current = np.int(np.mean(nonzeroy[good_right_inds]))
        else:
            line_flag = 4


        if line_flag ==3:
            for i in range(len(good_left_inds)):
                cv2.circle(out_img, (nonzerox[good_left_inds[i]], nonzeroy[good_left_inds[i]]), 1, (0, 255, 0), -1)
            for i in range(len(good_right_inds)):
                cv2.circle(out_img, (nonzerox[good_right_inds[i]], nonzeroy[good_right_inds[i]]), 1, (255,0, 0), -1)
            for window in range(0, nwindows):
                l_win_y_low = ly_current - (window + 1) * window_height
                l_win_y_high = ly_current - (window) * window_height
                l_win_x_low = lx_current - margin
                l_win_x_high = lx_current + margin
                r_win_y_low = ry_current - (window + 1) * window_height
                r_win_y_high = ry_current - (window) * window_height
                r_win_x_low = rx_current - margin
                r_win_x_high = rx_current + margin
                # draw rectangle
                # 0.33 is for width of the road
                cv2.rectangle(out_img, (l_win_x_low, l_win_y_low), (l_win_x_high, l_win_y_high), (0, 255, 0), 1)
                cv2.rectangle(out_img, (r_win_x_low, r_win_y_low), (r_win_x_high, r_win_y_high), (255,0, 0), 1)


                good_left_inds = ((nonzeroy >= l_win_y_low) & (nonzeroy < l_win_y_high) & (nonzerox >= l_win_x_low) & (
                            nonzerox < l_win_x_high)).nonzero()[0]
                good_right_inds = ((nonzeroy >= r_win_y_low) & (nonzeroy < r_win_y_high) & (nonzerox >= r_win_x_low) & (
                        nonzerox < r_win_x_high)).nonzero()[0]
                # check num of indicies in square and put next location to current
                if len(good_left_inds) > minpix:
                    lx_current = np.int(np.mean(nonzerox[good_left_inds]))
                if len(good_right_inds) > minpix:
                    rx_current = np.int(np.mean(nonzerox[good_right_inds]))


                # 338~344 is for recognize line which is yellow line in processed image(you can check in imshow)
                # if (l_win_y_low >= 338 and l_win_y_low < 344) and (r_win_y_low >= 338 and r_win_y_low < 344):
                #     # 0.165 is the half of the road(0.33)
                x_location = rx_current - lx_current + 75
                print('x',x_location)

        if line_flag != 4:
            # it's just for visualization of the valid inds in the region
            for i in range(len(good_left_inds)):
                cv2.circle(out_img, (nonzerox[good_left_inds[i]], nonzeroy[good_left_inds[i]]), 1, (0, 255, 0), -1)

            # window sliding and draw
            for window in range(0, nwindows):
                if line_flag == 1:
                    # rectangle x,y range init
                    win_y_low = y_current - (window + 1) * window_height
                    win_y_high = y_current - (window) * window_height
                    win_x_low = x_current - margin
                    win_x_high = x_current + margin
                    # draw rectangle
                    # 0.33 is for width of the road
                    cv2.rectangle(out_img, (win_x_low, win_y_low), (win_x_high, win_y_high), (0, 255, 0), 1)
                    # cv2.rectangle(out_img, (win_x_low + int(width * 0.33), win_y_low),
                    #               (win_x_high + int(width * 0.33), win_y_high), (255, 0, 0), 1)
                    # indicies of dots in nonzerox in one square
                    good_left_inds = ((nonzeroy >= win_y_low) & (nonzeroy < win_y_high) & (nonzerox >= win_x_low) & (
                                nonzerox < win_x_high)).nonzero()[0]
                    # check num of indicies in square and put next location to current
                    if len(good_left_inds) > minpix:
                        x_current = np.int(np.mean(nonzerox[good_left_inds]))

                    elif nonzeroy[left_lane_inds] != [] and nonzerox[left_lane_inds] != []:
                        p_left = np.polyfit(nonzeroy[left_lane_inds], nonzerox[left_lane_inds], 2)
                        x_current = np.int(np.polyval(p_left, win_y_high))
                    # 338~344 is for recognize line which is yellow line in processed image(you can check in imshow)
                    if win_y_low >= 338 and win_y_low < 344:
                        # 0.165 is the half of the road(0.33)
                        x_location = x_current + 180
                else:  # change line from left to right above(if)
                    win_y_low = y_current - (window + 1) * window_height
                    win_y_high = y_current - (window) * window_height
                    win_x_low = x_current - margin
                    win_x_high = x_current + margin
                    cv2.rectangle(out_img, (win_x_low - int(width * 0.33), win_y_low),
                                  (win_x_high - int(width * 0.33), win_y_high), (0, 255, 0), 1)
                    cv2.rectangle(out_img, (win_x_low, win_y_low), (win_x_high, win_y_high), (255, 0, 0), 1)
                    good_right_inds = ((nonzeroy >= win_y_low) & (nonzeroy < win_y_high) & (nonzerox >= win_x_low) & (
                                nonzerox < win_x_high)).nonzero()[0]
                    if len(good_right_inds) > minpix:
                        x_current = np.int(np.mean(nonzerox[good_right_inds]))
                    elif nonzeroy[right_lane_inds] != [] and nonzerox[right_lane_inds] != []:
                        p_right = np.polyfit(nonzeroy[right_lane_inds], nonzerox[right_lane_inds], 2)
                        x_current = np.int(np.polyval(p_right, win_y_high))
                    if win_y_low >= 338 and win_y_low < 344:
                        # 0.165 is the half of the road(0.33)
                        x_location = x_current - 250

                left_lane_inds.extend(good_left_inds)
            #        right_lane_inds.extend(good_right_inds)

            # left_lane_inds = np.concatenate(left_lane_inds)
            # right_lane_inds = np.concatenate(right_lane_inds)

            # else:
        return out_img, x_location


    def find_sliding_point(self,img):
        output = np.dstack((img, img, img))
        x_start_R = 0
        x_start_L = 0
        for i in range(440, 0, -5):
            l_win_y_low = 370
            l_win_y_high = 390
            l_win_x_low = i - 5
            l_win_x_high = i + 5
            cnt = 0
            cv2.rectangle(output, (l_win_x_low, l_win_y_low),
                          (l_win_x_high, l_win_y_high), (0, 255, 0), 1)
            for j in range(l_win_y_low,l_win_y_high):
                for k in range(l_win_x_low,l_win_x_high):
                    if img[j][k] > 10:
                        cnt +=1

            if cnt > 10:
                x_start_L = i
                cv2.rectangle(output, (l_win_x_low, l_win_y_low),
                              (l_win_x_high, l_win_y_high), (0,0,255), 1)
                break


        for i in range(440, 795, 5):
            r_win_y_low = 370
            r_win_y_high = 390
            r_win_x_low = i - 5
            r_win_x_high = i + 5
            cnt = 0
            cv2.rectangle(output, (r_win_x_low, r_win_y_low),
                          (r_win_x_high, r_win_y_high), (255, 0, 0), 1)
            for j in range(r_win_y_low,r_win_y_high):
                for k in range(r_win_x_low,r_win_x_high):
                    if img[j][k] > 10:
                        cnt +=1

            if cnt > 10:
                x_start_R = i
                cv2.rectangle(output, (r_win_x_low, r_win_y_low),
                              (r_win_x_high, r_win_y_high), (0, 0,255), 1)
                break


        cnt_left = 0
        cnt_right = 0
        cnt = 0
        if x_start_L is not None or x_start_R is not None:
            while (x_start_R - x_start_L < 300):
                if cnt > 2:
                    break
                if x_start_L > 210:
                    cnt_left += 1

                    for i in range(x_start_L - 5, 0, -5):
                        tmp_cnt = 0
                        l_win_y_low = 370
                        l_win_y_high = 390
                        l_win_x_low = i - 10
                        l_win_x_high = i + 10
                        cv2.rectangle(output, (l_win_x_low, l_win_y_low),
                                      (l_win_x_high, l_win_y_high), (0, 255, 0), 1)
                        for j in range(l_win_y_low, l_win_y_high):
                            for k in range(l_win_x_low, l_win_x_high):
                                if img[j][k] > 10:
                                    tmp_cnt += 1

                        if tmp_cnt > 10:
                                tmp_L = i
                                if tmp_L != 0:
                                    x_start_L = tmp_L
                                    cv2.rectangle(output, (l_win_x_low, l_win_y_low),
                                                  (l_win_x_high, l_win_y_high), (0, 0, 255), 1)
                                break
                if x_start_R < 580:
                    cnt_right += 1
                    for i in range(x_start_R + 5, 795,5):
                        tmp_cnt = 0
                        r_win_y_low = 370
                        r_win_y_high = 390
                        r_win_x_low = i - 10
                        r_win_x_high = i + 10
                        cv2.rectangle(output, (r_win_x_low, r_win_y_low),
                                      (r_win_x_high, r_win_y_high), (255, 0, 0), 1)
                        for j in range(r_win_y_low, r_win_y_high):
                            for k in range(r_win_x_low, r_win_x_high):
                                if img[j][k] > 10:
                                    tmp_cnt += 1

                        if tmp_cnt > 10:
                            tmp_R = i
                            if tmp_R != 0:
                                x_start_R = tmp_R
                                cv2.rectangle(output, (r_win_x_low, r_win_y_low),
                                              (r_win_x_high, r_win_y_high), (0, 0, 255), 1)
                            break
                cnt +=1
        # cv2.imshow('output',output)
        return x_start_L,x_start_R


    def unwarp(self,img):
        h = self.frame.shape[0]
        w = self.frame.shape[1]
        src = np.float32([[-60, 355],
                          [10, 400],
                          [w - 15, 405],
                          [w, 360]])

        dst = np.float32([[0, 0],
                          [0, h],
                          [w, h],
                          [w + 50, 0]])

        M = cv2.getPerspectiveTransform(dst,src)
        wap = cv2.warpPerspective(img, M, (640, 480), flags=cv2.INTER_LINEAR)

        return wap
    def main(self,img):
        x_start_l,x_start_r = self.findpoint.findpoint(img)
        output , x_location = self.sliding_window(x_start_l,x_start_r,img)
        return output, x_location
