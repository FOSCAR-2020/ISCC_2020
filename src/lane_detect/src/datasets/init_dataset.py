import os
import torch
import importlib
import subprocess
import math
import numpy as np
import random
import ref


def create(opt, split):
    info = exec(opt, ref.cacheFile)
    dataset = importlib.import_module('datasets.' + opt.dataset)
    print('opt.dataset : ', opt.dataset)
    #print('dataset.getInstance(info,opt,split) : ',dataset.getInstance(info, opt, split))
    return dataset.getInstance(info, opt, split)


def exec(opt, cacheFile):
    #print("type : ", type(str(opt.data)))
    #opt.data = str(opt.data)
    assert os.path.exists(str(opt.data)), 'Data directory not found: ' + opt.data

    print("=> Generating list of data")


    if opt.testOnly:
        dataFile = str(ref.data_root / 'v1.1' /  'test1.txt')
    else:
        dataFile = str(ref.data_root  / 'training' / 'train1.txt')
        print("dataFile Path : ", dataFile)

    with open(dataFile, 'r') as f:
        dataList = f.read().splitlines()
    dataList = [x[:-4] for x in dataList]

    #print("dataList : ", dataList)

    random.shuffle(dataList)
    # set train/val 4800/200

    print("opt.data : ", opt.data)

    image_path = opt.data / "gt_image"
    line_path = opt.data / "gt_binary_image"

    #print(image_path)
    #print(line_path)

    trainImg = [image_path / "{}.png".format(x) for x in dataList]
    #print(trainImg)
    trainLine = [line_path / "{}.png".format(x) for x in dataList]

    if opt.testOnly:
        val_list = dataList[:10]
        train_list = dataList[10:100]

        valImg = trainImg[:10]
        valLine = trainLine[:10]
        trainImg = trainImg[10:100]
        trainLine = trainLine[10:100]
    else:
        val_list = dataList[:300]
        train_list = dataList[300:]

        valImg = trainImg[:300]
        valLine = trainLine[:300]
        trainImg = trainImg[300:]
        trainLine = trainLine[300:]

    numTrain = len(trainImg)
    numVal = len(valImg)

    print('#Training images: {}'.format(numTrain))
    print('#Val images: {}'.format(numVal))

    info = {'basedir': opt.data,
            'train': {
                'imagePath'  : trainImg,
                'linePath'   : trainLine
                },
            'val': {
                'imagePath'  : valImg,
                'linePath'   : valLine
                }
            }

    torch.save(info, str(cacheFile))

    return info
