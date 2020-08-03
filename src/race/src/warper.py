import cv2
import numpy as np


class Warper:
    def __init__(self):
        h = 448
        w = 800
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
            [480*1.6, 530*1.3],
            [480*(-0.1), 530*1.3],
            [0, h*0.62],
            [w, h*0.62],
        ])
        dst = np.float32([
            [w * 0.65, h],
            [w * 0.35, h],
            [-300, 0],
            [w+300, 0],
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
