import cv2
import numpy as np


class Warper:
    def __init__(self):
        h = 480
        w = 640
        print("h : " ,h)
        print("w : " ,w)

        self.src = np.float32([
            [ 195, 75],
            [ 375, 75],
            [-100, 240],
            [640, 240],
        ])
        self.dst = np.float32([
            [0, 0],
            [w, 0],
            [0, h],
            [w, h],
        ])
        # dst = np.float32([
        #     [0, 0],
        #     [w, 0],
        #     [0, h],
        #     [w , h],
        # ])


        self.M = cv2.getPerspectiveTransform(self.src, self.dst)
        self.Minv = cv2.getPerspectiveTransform(self.dst, self.src)

    def warp(self, src_img, img):
        img_height, img_width = img.shape

        output_img = np.dstack((src_img,src_img,src_img))*255

        for i in range(len(self.src)):
            cv2.circle(output_img, (self.src[i][0], self.src[i][1]), 3, (255,255,0), -1)


        return output_img, cv2.warpPerspective(
            img,
            self.M,
            (img_width, img_height),
            flags=cv2.INTER_LINEAR
        )

    def unwarp(self, img):
        return cv2.warpPersective(
            img,
            self.Minv,
            (img.shape[1], img.shape[0]),
            flags=cv2.INTER_LINEAR
        )
