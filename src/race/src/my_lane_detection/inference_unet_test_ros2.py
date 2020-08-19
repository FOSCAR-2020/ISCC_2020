 # python inference_unet.py --netType Unet --GPUs 0 --batchSize 1
import matplotlib.pylab as plt
import sys
from multiprocessing import Process
#print(sys.path)
#sys.path.append('/lib/python3.7/site-packages')
import opts
import math
import importlib
import os

import _init_paths
import torch
#import torchvision.transforms as transforms
from torch.autograd import Variable

import cv2
import numpy as np
from PIL import Image

import rospy

import time
from warper import Warper
#from slidewindow import SlideWindow
# from slidewindow_ver1 import LineDetector
from slidewindow_ver2 import LineDetector

from pidcal import PidCal
from race.msg import drive_values

ack_publisher = None
car_run_speed = 0
max_speed = 3
slidewindow_middle=400
rospy.init_node("lane_detect")
pid_draw=[]
fps_list = []
drive_values_pub = rospy.Publisher('control_value', drive_values, queue_size=1)

def tanh_preprocess(pid):
    print("PID : {}".format(pid))
    Direction_Degree=2000*np.tanh(1/10*(pid))

    if Direction_Degree > 2000:
        Direction_Degree = 1999
    elif Direction_Degree < -2000:
        Direction_Degree = -1999

    # -1999 < Direction_Degree < +1999
    return Direction_Degree

def linear_preprocess(pid):
    if -0.64 < pid and pid < 0.64:
        Direction_Degree = 5**5*pid/2
    elif -0.64 >= pid:
        Direction_Degree = 1000
    elif 0.64 <= pid:
        Direction_Degree = 1000

    # -1999 < Direction_Degree < +1999
    return Direction_Degree
def linear_x_loc(x_loc):
    if x_loc is None: x_loc = slidewindow_middle

    Direction_Degree = 2000/slidewindow_middle*(x_loc-slidewindow_middle)
    return Direction_Degree

    # -1999 < Direction_Degree < +1999
    return Direction_Degree
def auto_drive(pid, x_loc = None):
    global car_run_speed
    global max_speed

    w = 0

    if car_run_speed < max_speed:
        car_run_speed += 0.01

    drive_value = drive_values()

    drive_value.throttle = int(8)
    #drive_value.steering = tanh_preprocess(pid)
    #drive_value.steering = linear_preprocess(pid)
    drive_value.steering = linear_x_loc(x_loc)

    drive_values_pub.publish(drive_value)

    print("steer : ", drive_value.steering)
    print("throttle : ", drive_value.throttle)


def normalize(ipt, mean, std):
    ipt[:][:][0] = (ipt[:][:][0] - mean[0]) / std[0]
    ipt[:][:][1] = (ipt[:][:][1] - mean[1]) / std[1]
    ipt[:][:][2] = (ipt[:][:][2] - mean[2]) / std[2]
    return ipt

def unNormalize(ipt, mean, std):
    ipt[:][:][0] = (ipt[:][:][0] * std[0]) + mean[0]
    ipt[:][:][1] = (ipt[:][:][1] * std[1]) + mean[1]
    ipt[:][:][2] = (ipt[:][:][2] * std[2]) + mean[2]
    return ipt

def preprocess_img(im):
    mean = np.array([0.485, 0.456, 0.406])
    std = np.array([0.229, 0.224, 0.225])
    im = np.asarray(im)
    im = normalize(im, mean, std)
    im = np.transpose(im, (2, 0, 1))
    return im

def postprocess_img(im):
    mean = np.array([0.485, 0.456, 0.406])
    std = np.array([0.229, 0.224, 0.225])
    im = np.transpose(im, (1, 2, 0))
    im = unNormalize(im, mean, std)
    return im

def main():
    os.environ["CUDA_VISIBLE_DEVICES"] = '0'
    #cudnn.benchmark = True

    opt = opts.parse()
    warper = Warper()
    #slidewindow  = SlideWindow()
    # slidewindow  = LineDetector()

    pidcal = PidCal()

    print(("device id: {}".format(torch.cuda.current_device())))
    print("torch.version",torch.__version__)
    print("cuda_version",torch.version.cuda)


    models = importlib.import_module('models.init')
    # print(models)
    criterions = importlib.import_module('criterions.init')
    checkpoints = importlib.import_module('checkpoints')
    Trainer = importlib.import_module('models.' + opt.netType + '-train')

    # Data loading
    print('=> Setting up data loader')
    #trainLoader, valLoader = DataLoader.create(opt)
    #print('opt',opt)

    # Load previous checkpoint, if it exists
    print('=> Checking checkpoints')
    checkpoint = checkpoints.load(opt)

    # Create model
    model, optimState = models.setup(opt, checkpoint)
    model.cuda()

    criterion = criterions.setup(opt, checkpoint, model)

    ##################################################################################
    model.eval()

    cap = cv2.VideoCapture("/home/foscar/ISCC_2019/src/race/src/my_lane_detection/input_video/0.avi")
    ret, frame = cap.read()
    slidewindow  = LineDetector(frame)
    if cap.isOpened():
        print("width : {}, height : {}".format(cap.get(3), cap.get(4)))

    video_width = int(cap.get(3))
    video_height = int(cap.get(4))

    fourcc = cv2.VideoWriter_fourcc(*'DIVX')
    video_name = time.time()
    out = cv2.VideoWriter('output_video/{}.avi'.format(video_name), fourcc, 25.0, (video_width,video_height),0)

    prev_time = 0


    count=0

    while True:
        ret, frame = cap.read()
        count+=1
        if ret:
            cur_time = time.time()
            frame = cv2.resize(frame, (480,360))

            input_img = frame / 255.
            input_img = preprocess_img(input_img)

            # array to tensor
            input_img = torch.from_numpy(input_img).float()

            with torch.no_grad():
                inputData_var = Variable(input_img).unsqueeze(0).cuda()

                # inference
                output = model.forward(inputData_var)

                print("output.shape : ", output.shape)

                # gpu -> cpu,  tensor -> numpy
                output = output.detach().cpu().numpy()

                output = output[0]

                output = postprocess_img(output)
                # cv2.imshow("203",output)
                output *= 255
                output = np.clip(output, 0, 255)
                output = np.uint8(output)

                # resize
                output = cv2.resize(output, (640, 480))
                # cv2.imshow('resize',output)
                # threshold
                ret, thr_img = cv2.threshold(output, 20, 255, 0)
                # cv2.imshow('threshold',thr_img)
                # warp
                warp_img = warper.warp( thr_img)

                # cv2.imshow('warped',warp_img)
                # cv2.imshow("new output", canny_like_output)

                #canny = cv2.Canny(warp_img, 40, 255)
                kernel1 = np.array([[-1, -1, -1], [-1, 9, -1], [-1, -1, -1]])
                kernel2 = np.ones((5, 5,), np.uint8)

                #dilate = cv2.dilate(warp_img, kernel1, iterations=2)
                #closed = cv2.morphologyEx(dilate, cv2.MORPH_OPEN, kernel2)
                # x_start_L, x_start_R=slidewindow.find_sliding_point(warp_img)
                # img, x_location = slidewindow.slide_window(x_start_L,x_start_R,warp_img)
                slided_img, x_location,point_list_left,point_list_right = slidewindow.main(warp_img)

                if x_location != None:
                    # cv2.circle(img,(int(x_location),300),5,(0,0,255),3)
                    pid = round(pidcal.pid_control(int(x_location)), 6)
                    #print("pid rate : ", pid)
                    auto_drive(pid, x_location)
                else:
                    pid = pidcal.pid_control(slidewindow_middle)
                    print("pid rate : ", pid)
                    auto_drive(pid)


                end_time = time.time()
                sec = end_time - cur_time

                fps = 1/sec
                fps_list.append(fps)

                print("Estimated fps {0} " . format(fps))

                out.write(output)

                cv2.imshow("src", frame)
                pid_draw.append(pid)

                # cv2.imshow("th_img", thr_img)
                # cv2.imshow("output", output)
        # img = cv2.imread('/home/foscar/Downloads/wapped_screenshot_12.08.2020.png',cv2.IMREAD_GRAYSCALE)
        # img = cv2.resize(img, (640, 480))
        # ret, thr_img = cv2.threshold(img,20,255,cv2.THRESH_BINARY)
            # img ,xloc, point_list_left, point_list_right = slidewindow.main(thr_img)
        # plt.xlim(0,640)
        # plt.ylim(0,480)
        # plt.plot(point_list_left[0], point_list_left[1])
        # plt.plot(point_list_right[0], point_list_right[1])
        # plt.show()
        # cv2.imshow('aa',img)
        # key = cv2.waitKey(1) & 0xFFx
        # if key == 27: break
        # elif key == ord('p'):
        #     cv2.waitKey(-1)
                cv2.imshow("ws", slided_img)
                print("x_loc :",x_location)
                key = cv2.waitKey(1) & 0xFF
                if key == 27: break
                elif key == ord('p'):
                    cv2.waitKey(-1)


if __name__ == '__main__':
    main()
