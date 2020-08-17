import cv2
import numpy as np
import matplotlib.pyplot as plt
from findpoint import FindPoint


class LineDetector:
    def __init__(self,img):

        self.frame = None
        self.leftx = None
        self.rightx = None
        # self.output = None
        self.frame = 0
        self.frame_list = []
        self.findpoint = FindPoint(img)

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

        point_list_left = list()
        point_list_right = list()

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
        # lx_current = 120
        # ly_current = 350
        # rx_current = 550
        # ry_current = 350
        if len(good_left_inds) > minpix and len(good_right_inds) > minpix:
            line_flag = 3
            lx_current = np.int(np.mean(nonzerox[good_left_inds]))
            ly_current = np.int(np.mean(nonzeroy[good_left_inds]))
            rx_current = np.int(np.mean(nonzerox[good_right_inds]))
            ry_current = np.int(np.mean(nonzeroy[good_right_inds]))

        elif len(good_left_inds) > minpix:
            line_flag = 1
            lx_current = np.int(np.mean(nonzerox[good_left_inds]))
            ly_current = np.int(np.mean(nonzeroy[good_left_inds]))
            rx_current = None
            ry_current = None
            max_y = y_current
        elif len(good_right_inds) > minpix:
            line_flag = 2
            rx_current = nonzerox[good_right_inds[np.argmax(nonzeroy[good_right_inds])]]
            ry_current = np.int(np.max(nonzeroy[good_right_inds]))
            lx_current = None
            ly_current = None
        else:
            line_flag = 4
            # rx_current
            # ry_current
        # if line_flag ==3:
        #     for i in range(len(good_left_inds)):
        #         cv2.circle(out_img, (nonzerox[good_left_inds[i]], nonzeroy[good_left_inds[i]]), 1, (0, 255, 0), -1)
        #     for i in range(len(good_right_inds)):
        #         cv2.circle(out_img, (nonzerox[good_right_inds[i]], nonzeroy[good_right_inds[i]]), 1, (255,0, 0), -1)
        #     for window in range(0, nwindows):

                # print('x',x_location)

        if line_flag != 4:
            # it's just for visualization of the valid inds in the region
            for i in range(len(good_left_inds)):
                cv2.circle(out_img, (nonzerox[good_left_inds[i]], nonzeroy[good_left_inds[i]]), 1, (0, 255, 0), -1)
            for i in range(len(good_right_inds)):
                cv2.circle(out_img, (nonzerox[good_right_inds[i]], nonzeroy[good_right_inds[i]]), 1, (255,0, 0), -1)
            # window sliding and draw
            # print(lx_current)
            # print(rx_current)
            for window in range(0, nwindows):
                # if lx_current and rx_current:
                #     # print(line_flag)
                #     cv2.circle(out_img,(lx_current,ly_current-window*window_height-3),3,(0,0,255),-1)
                #     cv2.circle(out_img,(rx_current,ry_current-window*window_height-3),3,(0,0,255),-1)
                #     mean_x = (lx_current + rx_current)/2
                # cv2.circle(out_img,(mean_x,ry_current-window*window_height-3),3,(0,255,255),-1)
                # point_list_left.append((lx_current, ly_current-window*window_height-3))
                # point_list_right.append((rx_current,ry_current-window*window_height-3))
                if lx_current and  rx_current:
                    cv2.circle(out_img,(lx_current,ly_current-window*window_height-3),3,(0,0,255),-1)
                    cv2.circle(out_img,(rx_current,ry_current-window*window_height-3),3,(0,0,255),-1)
                    mean_x = (lx_current + rx_current)/2
                    cv2.circle(out_img,(mean_x,ry_current-window*window_height-3),3,(0,255,255),-1)
                    point_list_left.append((lx_current, ly_current-window*window_height-3))
                    point_list_right.append((rx_current,ry_current-window*window_height-3))
                elif lx_current:
                    cv2.circle(out_img,(lx_current,ly_current-window*window_height-3),3,(0,0,255),-1)
                    mean_x = (lx_current + width/2)
                    cv2.circle(out_img,(mean_x,ly_current-window*window_height-3),3,(0,255,255),-1)
                    point_list_left.append((lx_current, ly_current-window*window_height-3))
                elif rx_current:
                    # cv2.circle(out_img,(lx_current,ly_current-window*window_height-3),3,(0,0,255),-1)
                    cv2.circle(out_img,(rx_current,ry_current-window*window_height-3),3,(0,0,255),-1)
                    mean_x = (rx_current-width/2)/2
                    cv2.circle(out_img,(mean_x,ry_current-window*window_height-3),3,(0,255,255),-1)
                    # point_list_left.append((lx_current, ly_current-window*window_height-3))
                    point_list_right.append((rx_current,ry_current-window*window_height-3))
                if line_flag == 3:
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
                elif line_flag == 1:
                    # rectangle x,y range init
                    win_y_low = ly_current - (window + 1) * window_height
                    win_y_high = ly_current - (window) * window_height
                    win_x_low = lx_current - margin
                    win_x_high = lx_current + margin
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
                        # x_current = np.int(np.mean(nonzerox[good_left_inds]))
                        lx_current = np.int(np.mean(nonzerox[good_left_inds]))

                    # elif nonzeroy[left_lane_inds] != [] and nonzerox[left_lane_inds] != []:
                    #     p_left = np.polyfit(nonzeroy[left_lane_inds], nonzerox[left_lane_inds], 2)
                    #     x_current = np.int(np.polyval(p_left, win_y_high))
                    # # 338~344 is for recognize line which is yellow line in processed image(you can check in imshow)
                    # if win_y_low >= 338 and win_y_low < 344:
                    #     # 0.165 is the half of the road(0.33)
                    #     x_location = x_current + 180



                elif line_flag ==2:
                    win_y_low = ry_current - (window + 1) * window_height
                    win_y_high = ry_current - (window) * window_height
                    win_x_low = rx_current - margin
                    win_x_high = rx_current + margin
                    # cv2.rectangle(out_img, (win_x_low , win_y_low),
                                  # (win_x_high, win_y_high), (0, 255, 0), 1)
                    cv2.rectangle(out_img, (win_x_low, win_y_low), (win_x_high, win_y_high), (255, 0, 0), 1)
                    good_right_inds = ((nonzeroy >= win_y_low) & (nonzeroy < win_y_high) & (nonzerox >= win_x_low) & (
                                nonzerox < win_x_high)).nonzero()[0]
                    if len(good_right_inds) > minpix:
                        # x_current = np.int(np.mean(nonzerox[good_right_inds]))
                        rx_current = np.int(np.mean(nonzerox[good_right_inds]))

                    # elif nonzeroy[right_lane_inds] != [] and nonzerox[right_lane_inds] != []:
                    #     p_right = np.polyfit(nonzeroy[right_lane_inds], nonzerox[right_lane_inds], 2)
                    #     x_current = np.int(np.polyval(p_right, win_y_high))
                    # if win_y_low >= 338 and win_y_low < 344:
                    #     # 0.165 is the half of the road(0.33)
                    #     x_location = x_current - 250

                # left_lane_inds.extend(good_left_inds)
            #        right_lane_inds.extend(good_right_inds)

            # left_lane_inds = np.concatenate(left_lane_inds)
            # right_lane_inds = np.concatenate(right_lane_inds)

            # else:
        return out_img, x_location, point_list_left, point_list_right



    def main(self,img):

        x_start_l,x_start_r = self.findpoint.findpoint(img)
        output , x_location, point_list_left, point_list_right = self.sliding_window(x_start_l,x_start_r,img)
        return output, x_location, point_list_left, point_list_right
