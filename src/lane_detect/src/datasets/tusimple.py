from torch.utils.data.dataset import *
import torch
import os
import numpy as np
import cv2
import datasets.transforms as t
import pickle

class TusimpleDist(Dataset):
    def __init__(self, imageInfo, opt, split):
        self.imageInfo = imageInfo[split]
        self.opt = opt
        self.split = split
        self.dir = imageInfo['basedir']

    def __getitem__(self, index):
        # RGB Image Path
        imgPath = self.imageInfo['imagePath'][index]
        # Posix to str
        imgPath = str(imgPath)

        # load RGB Image
        rgbImage = cv2.imread(imgPath)
        rgbImage = cv2.resize(rgbImage,(640,480))

        # 0 ~ 255 to 0 ~ 1
        rgbImage = rgbImage / 255.
        rgbImage = self.preprocess(rgbImage)

        # Line Image path
        linePath = self.imageInfo['linePath'][index]
        # Posix to str
        linePath = str(linePath)

        # load Line image
        lineImage = cv2.imread(linePath,0)
        lineImage = cv2.resize(lineImage,(640,480))
        #print("height : ", lineImage.shape[0])
        #print("width : ", lineImage.shape[1])

        lineImage = self.preprocessLine(lineImage)

        # Seg Image path
        #segPath = self.imageInfo['segPath'][index]
        # Posix to str
        #segPath = str(segPath)

        # load Seg image
        #segImage = cv2.imread(segPath)[:,:,0]

        #segImage = cv2.resize(segImage,(512,256))
        # cv2.imshow("segImage", segImage)
        # cv2.waitKey(1000000)


        # cv2.imshow("segImage", segImage)
        # cv2.waitKey(1000000)

        #segImage = self.preprocessSeg(segImage)

        # convert numpy to tensor
        image = torch.from_numpy(rgbImage).float()
        line = torch.from_numpy(lineImage).float()
        #seg = torch.from_numpy(segImage).float()

        imgName = imgPath.split('/')[-1]

        return image, line, imgName


    def __len__(self):
        return len(self.imageInfo['imagePath'])

    def preprocess(self, im):
        mean = np.array([0.485, 0.456, 0.406])
        std = np.array([0.229, 0.224, 0.225])
        im = np.asarray(im)
        # normalize
        im = t.normalize(im, mean, std)
        # (width, height, channel) to (channel, width, height)
        im = np.transpose(im, (2,0,1))
        return im

    def preprocessLine(self, line):
        line = np.asarray(line)
        tmp = np.zeros((1,480, 640))
        tmp[0, :, :] = line
        line = tmp
        return line

    def preprocessSeg(self, seg):
        #for label_pix in unique_arr:
        seg = np.asarray(seg)
        tmp = np.zeros((1, 480, 640))
        tmp[0, :, :] = seg
        seg = tmp
        return seg


def getInstance(info, opt, split):
    instance = TusimpleDist(info, opt, split)
    return instance
