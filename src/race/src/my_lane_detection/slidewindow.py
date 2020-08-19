import numpy as np
import cv2
from scipy.interpolate import *
from matplotlib.pyplot import *
from sklearn.linear_model import LinearRegression
import math

class SlideWindow:
    def __init__(self):
        self.left_fit = None
        self.right_fit = None
        self.leftx = None
        self.rightx = None

    def w_slidewindow(self, img):
        height, width = img.shape

        # print("Image Width : {}   Image Height : {}".format(width, height))

        roi_img = img[height-150:height-100,:].copy()

        roi_height, roi_width = roi_img.shape

        # print("ROI Width : {}   ROI Height : {}".format(roi_width, roi_height))

        cf_img = np.dstack((roi_img,roi_img,roi_img))

        window_height = 20
        window_width = 30

        # minpix : 30% number of total window pixel
        minpix = window_height*window_width // 3
        n_windows = roi_width//window_width//2
        # 480 // 30 // 2 == 8
        # pts_left = np.array([[roi_width//2, roi_height//2-window_height//2], [roi_width//2, roi_height//2+window_height//2], [roi_width//2-window_width, roi_height//2+window_height//2],[roi_width//2-window_width, roi_height//2-window_height//2]], np.int32)
        # cv2.polylines(cf_img,[pts_left],False,(0,255,0),1)
        # pts_right = np.array([[roi_width//2, roi_height//2-window_height//2], [roi_width//2, roi_height//2+window_height//2], [roi_width//2+window_width, roi_height//2+window_height//2],[roi_width//2+window_width, roi_height//2-window_height//2]], np.int32)
        # cv2.polylines(cf_img,[pts_right],False,(0,0,255),1)
        pts_center = np.array([[roi_width//2,0],[roi_width//2, roi_height]], np.int32)
        cv2.polylines(cf_img, [pts_center],False, (0,120,120),1)

        nonzero = roi_img.nonzero()
        nonzeroy = np.array(nonzero[0])
        nonzerox = np.array(nonzero[1])

        x_center = roi_width//2 # 240
        y_center = roi_height//2 # 160

        left_idx = 0
        right_idx = 0

        find_left = False
        find_right = False

        left_start_x = None
        left_start_y = None

        right_start_x = None
        right_start_y = None

        dist_threshold = 120
        dist = None

        for i in range(0,n_windows):
            if find_left is False:
                win_left_y_low = y_center - window_height//2
                win_left_y_high = y_center + window_height//2

                win_left_x_high = x_center - left_idx*window_width
                win_left_x_low = x_center - (left_idx+1)*window_width

            if find_right is False:
                win_right_y_low = y_center - window_height//2
                win_right_y_high = y_center + window_height//2

                win_right_x_low = x_center + right_idx*window_width
                win_right_x_high = x_center + (right_idx+1)*window_width
            #print(win_left_y_low, ' ', win_left_x_low, ' ', win_left_y_high, ' ', win_left_x_high )

            cv2.rectangle(cf_img, (win_left_x_low, win_left_y_low), (win_left_x_high, win_left_y_high), (0,255,0), 1)
            cv2.rectangle(cf_img, (win_right_x_low, win_right_y_low), (win_right_x_high, win_right_y_high), (0,0,255), 1)

            good_left_inds = ((nonzeroy >= win_left_y_low) & (nonzeroy < win_left_y_high) & (nonzerox >= win_left_x_low) & (nonzerox < win_left_x_high)).nonzero()[0]
            good_right_inds = ((nonzeroy >= win_right_y_low) & (nonzeroy < win_right_y_high) & (nonzerox >= win_right_x_low) & (nonzerox < win_right_x_high)).nonzero()[0]

            if len(good_left_inds) > minpix and find_left is False:
                find_left = True

                left_start_x = np.int(np.mean(nonzerox[good_left_inds]))
                left_start_y = roi_height//2

                for i in range(len(good_left_inds)):
                    cv2.circle(cf_img, (nonzerox[good_left_inds[i]], nonzeroy[good_left_inds[i]]), 1, (0,255,0), -1)
            else:
                left_idx += 1



            if len(good_right_inds) > minpix and find_right is False:
                find_right = True

                right_start_x = np.int(np.mean(nonzerox[good_right_inds]))
                right_start_y = roi_height//2

                for i in range(len(good_right_inds)):
                    cv2.circle(cf_img, (nonzerox[good_right_inds[i]], nonzeroy[good_right_inds[i]]), 1, (0,0,255), -1)
            else:
                right_idx += 1


            if left_start_x is not None and right_start_x is not None:
                dist = right_start_x - left_start_x

                if dist_threshold < dist and dist < dist_threshold + 80:
                    cv2.circle(cf_img, (right_start_x, right_start_y),3, (255,0,0),-1)
                    cv2.circle(cf_img, (left_start_x, left_start_y), 3, (255,0,0),-1)

                    return True, left_start_x, right_start_x, cf_img

        return False, left_start_x, right_start_x, cf_img


    def h_slidewindow(self, img, x_left_start, x_right_start):
        line_fitter = LinearRegression()
        h, w = img.shape
        output_img = np.dstack((img,img,img))
        nonzero = img.nonzero()
        nonzeroy = np.array(nonzero[0])
        nonzerox = np.array(nonzero[1])
        x_left = x_left_start
        x_right = x_right_start
        y_start = h/2 + 100
        window_height = 30
        window_width = 26

        find_left = False
        find_right = False

        minpix = window_width * window_height / 30
        n_windows = 5
        # print('nwindows',n_windows)
        left_idx = 0
        right_idx = 0
        center_x = np.zeros((5),dtype = 'f')
        center_y = np.zeros((5),dtype = 'f')
        center=np.zeros((5,2),dtype='f')

        size_center=0
        for i in range(n_windows):
            cv2.rectangle(output_img, (x_left - window_width, y_start - i * window_height), (x_left+ window_width, y_start - (i + 1) * window_height), (0,255,255),1)
            cv2.rectangle(output_img, (x_right - window_width, y_start - i * window_height), (x_right + window_width, y_start - (i + 1) * window_height), (255,0,255),1)
            good_left_inds = ((nonzerox >= x_left - window_width) & (nonzerox < x_left+ window_width) & (nonzeroy < y_start - i * window_height) & (nonzeroy >= y_start - (i + 1) * window_height)).nonzero()[0]
            good_right_inds = ((nonzerox >= x_right -window_width) & (nonzerox <= x_right + window_width) & (nonzeroy < y_start - i * window_height) & (nonzeroy >= y_start - (i + 1) * window_height)).nonzero()[0]

            # for j in range(len(good_left_inds)):
            #     cv2.circle(output_img, (nonzerox[good_left_inds[j]], nonzeroy[good_left_inds[j]]), 1, (0,255,0), -1)
            #
            # for j in range(len(good_right_inds)):
            #     cv2.circle(output_img, (nonzerox[good_right_inds[j]], nonzeroy[good_right_inds[j]]), 1, (0,0,255), -1)

            if len(good_left_inds) > minpix:
                # find_left = True
                x_left = np.int(np.mean(nonzerox[good_left_inds]))

            if len(good_right_inds) > minpix:
                # find_right = True
                x_right = np.int(np.mean(nonzerox[good_right_inds]))

            center_point = np.int((x_left +x_right)/2)
            if i < 5:
                # center[i] = np.array([center_point,y_start - i*window_height - 10])
                center_x[i] = np.array([center_point])
                center_y[i] = np.array([y_start - i*window_height - 10])
                size_center+=1

            # print('center',center_point)
            # print('y',y_start - (i*window_height) -3)
            # cv2.circle(output_img, (center_point,y_start - i*window_height - 10 ),3,(0,0,255),1)
        fp1 = np.polyfit(center_y,center_x,1)
        f1 = np.poly1d(fp1)
        returns_x = f1(center_y)
        cv2.line(output_img,(returns_x[0],center_y[0]),(returns_x[size_center-1],center_y[size_center-1]),(0,0,255))
        for i in range(5):
            cv2.circle(output_img,(returns_x[i],center_y[i]),3,(0,0,255),-1)
        cv2.circle(output_img,(center_x[0],center_y[0]),9,(255,50,0),-1)
        steer_theta=math.degrees( math.atan( (center_x[0]-returns_x[size_center-1] )/(center_y[size_center-1]-center_y[0]) ) )

        # line_fitter.fit(center.reshape(-1,1),center_y)
        # y_predicted = line_fitter.predict(center_x)

        return x_left_start, x_right_start, output_img ,steer_theta
