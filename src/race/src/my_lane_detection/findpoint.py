import numpy as np
import cv2


class FindPoint:
    def __init__(self):
        self.window_height = 5
        self.nwindows = 30
        self.margin = 20
        self.minpix = 70

    def findpoint(self, img):
        out_img = np.dstack((img, img, img))
        h, w = img.shape

        good_left_inds = []
        good_right_inds = []

        nonzero = img.nonzero()
        nonzerox = nonzero[1]
        nonzeroy = nonzero[0]
        tmp_lx = 0
        tmp_rx = 0
        for i in range(0, 256):
            win_high = 390
            win_low = 380
            l_x_max = w/2 - (i * 5 - 5)
            l_x_min = w/2 - (i * 5 + 5)
            good_left_inds = \
            ((nonzerox >= l_x_min) & (nonzeroy >= win_low) & (nonzeroy <= win_high) & (nonzerox <= l_x_max)).nonzero()[
                0]

            if len(good_left_inds) > self.minpix:
                tmp_lx = np.int(np.mean(nonzerox[good_left_inds]))
            cv2.rectangle(out_img, (l_x_max, 380), (l_x_min, 390), (0, 255, 0), 1)
            if tmp_lx != 0:
                break


        for i in range(0, 64):
            win_high = 390
            win_low = 380
            r_x_min = w/2 + (i * 5 - 5)
            r_x_max = w/2 + (i * 5 + 5)
            good_right_inds = \
                ((nonzerox >= r_x_min) & (nonzeroy >= win_low) & (nonzeroy <= win_high) & (
                            nonzerox <= r_x_max)).nonzero()[
                    0]

            if len(good_right_inds) > self.minpix:
                tmp_rx = np.int(np.mean(nonzerox[good_right_inds]))
            cv2.rectangle(out_img, (r_x_min, 380), (r_x_max, 390), (255, 0, 0), 1)
            if tmp_rx != 0:
                break
        if tmp_rx - tmp_lx < 300:
            for window in range(0,self.nwindows):
                if tmp_lx != 0:
                    l_x_min = tmp_lx-(window+1)*self.window_height
                    l_x_max = tmp_lx - (window) * self.window_height
                    good_left_inds = \
                        ((nonzerox >= l_x_min) & (nonzeroy >= win_low) & (nonzeroy <= win_high) & (
                                    nonzerox <= l_x_max)).nonzero()[
                            0]
                    if len(good_left_inds) > self.minpix:
                        tmp_lx = np.int(np.mean(nonzerox[good_left_inds]))
                        cv2.rectangle(out_img, (l_x_max, 380), (l_x_min, 390), (0, 255, 0), 1)
                if tmp_rx != 0:
                    r_x_max = tmp_rx-(window+1)*self.window_height
                    r_x_min = tmp_rx - (window) * self.window_height
                    good_right_inds = \
                        ((nonzerox >= r_x_min) & (nonzeroy >= win_low) & (nonzeroy <= win_high) & (
                                    nonzerox <= r_x_max)).nonzero()[
                            0]
                    if len(good_right_inds) > self.minpix:
                        tmp_rx = np.int(np.mean(nonzerox[good_left_inds]))
                        cv2.rectangle(out_img, (r_x_min, 380), (r_x_max, 390), (255,0, 0), 1)
        # print('l', tmp_lx , '   ', 'r',tmp_rx)
        cv2.rectangle(out_img, (tmp_lx-5, 380), (tmp_lx+5, 390), (0, 0,255), 1)
        cv2.rectangle(out_img, (tmp_rx-5, 380), (tmp_rx+5, 390), (0,0,255), 1)
        # cv2.imshow('width_slide',out_img)
        return tmp_lx, tmp_rx