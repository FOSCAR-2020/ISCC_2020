import cv2
import numpy as np


def stoping(img):
    cv2.rectangle(img, (0, 220), (480, 270), (255, 0, 0), 2)
    stop_roi = cv2.cvtColor(img[220:270, :], cv2.COLOR_BGR2GRAY)

    rho = 1
    theta = np.pi / 180
    threshold = 50
    minLineLength = 100
    maxLineGap = 50
    houghlines = cv2.HoughLinesP(stop_roi, rho, theta, threshold, minLineLength, maxLineGap)

    color = [255, 0, 0]
    thickness = 2

    grads = []

    for line in houghlines:
        for x1, y1, x2, y2 in line:
            if ((x2 - x1) != 0):
                gradiant = (y2 - y1) / (x2 - x1)
                grads.append(gradiant)
                cv2.line(img[220:270, :], (x1, y1), (x2, y2), color, thickness)
    # print(len(grads))
    if (len(grads) >= 18):
        cv2.waitKey(-1)

    return


def stoping_tmp(img):
    cv2.rectangle(img, (0, 220), (480, 270), (255, 0, 0), 2)
    stop_roi = img[50:150, :]
    cv2.imshow('roi',stop_roi)
    nonzero = stop_roi.nonzero()
    print(len(nonzero[0]))
    if len(nonzero[0]) > 13000:
        cv2.waitKey(-1)



    return

def stop(img):
    gray = img
    # gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    stop_roi = gray[240:280,100:-100]
    cv2.rectangle(img, (150, 250), (310, 270), (255, 0, 0), 2)
    cv2.imshow('roi',stop_roi)
    nonzero = stop_roi.nonzero()
    print(len(nonzero[0]))
    if (len(nonzero[0]) > 2000):
        cv2.waitKey(-1)

    return
