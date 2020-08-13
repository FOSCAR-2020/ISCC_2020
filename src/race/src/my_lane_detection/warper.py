import cv2
import numpy as np


class Warper:
    def __init__(self):
        h = 480
        w = 640
        print("h : " ,h)
        print("w : " ,w)

        # distort scr to dst
        # src = np.float32([
        #     [w * 1.6, h * 1.3],
        #     [w * (-0.1), h * 1.3],
        #     [0, h * 0.62],
        #     [w, h * 0.62],
        # ])
        src = np.float32([
            [60,355],
            [10,400],
            [w-15,405],
            [w,360],
        ])
        dst = np.float32([
            [0,0],
            [0, h],
            [w, h],
            [w+50, 0],
        ])
        # dst = np.float32([
        #     [0, 0],
        #     [w, 0],
        #     [0, h],
        #     [w , h],
        # ])


        self.M = cv2.getPerspectiveTransform(src, dst)
        self.Minv = cv2.getPerspectiveTransform(dst, src)

    def warp(self, img):
        return cv2.warpPerspective(
            img,
            self.M,
            (img.shape[1], img.shape[0]),
            flags=cv2.INTER_LINEAR
        )

    def unwarp(self, img):
        return cv2.warpPersective(
            img,
            self.Minv,
            (img.shape[1], img.shape[0]),
            flags=cv2.INTER_LINEAR
        )
