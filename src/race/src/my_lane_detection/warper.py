import cv2
import numpy as np


class Warper:
    def __init__(self):
        self.h = 320
        self.w = 480
        print("h : " ,self.h)
        print("w : " ,self.w)

        # distort scr to dst
        self.src = np.float32([
            [175, 275],
            [475, 275],
            [0, 320],
            [640, 320],
        ])
        self.dst = np.float32([
            [120, 0],
            [self.w - 120 , 0],
            [120, self.h-50],
            [self.w-120, self.h-50],
        ])


        self.M = cv2.getPerspectiveTransform(self.src, self.dst)
        self.Minv = cv2.getPerspectiveTransform(self.dst, self.src)

    def warp(self, img):
        return cv2.warpPerspective(
            img,
            self.M,
            (self.w, self.h),
            flags=cv2.INTER_LINEAR
        ), img

    def unwarp(self, img):
        return cv2.warpPersective(
            img,
            self.Minv,
            (self.w, self.h),
            flags=cv2.INTER_LINEAR
        )
