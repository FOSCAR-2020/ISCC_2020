# 2020.06.25
# Lane Detection
import os
import sys
import opts
import math
import importlib
import _init_paths

# pytorch 1.14
import torch
import torchvision.transforms as transforms
from torch.autograd import Variable
# image processing lib
from PIL import Image
import cv2
import numpy as np

# python3 main.py --netType stackedHGB --GPUs 0 --LR 0.001 --batchSize 1 --nStack 7 --optim Adam

def main():
    os.environ["CUDA_VISIBLE_DEVICES"] = '0'

    opt = opts.parse()

    # Device ID : 0
    print("Device ID : {}".format(torch.cuda.current_device()))
    # torch.version : 1.4.0
    print("torch.version : ", torch.__version__)
    # cuda_version : 10.1
    print("cuda_version : ", torch.version.cuda)

    models = importlib.import_module('models.init')
    criterions = importlib.import_module('criterions.init')
    checkpoints = importlib.import_module('checkpoints')
    #Trainer = importlib.import_module('models.' + opt.netType  '-train')
    Trainer = importlib.import_module('models.stackedHGB-train')

    try:
        DataLoader = importlib.import_module('models.stackedHGB-dataloader')
        #print("try 문")
    except ImportError:
        DataLoader = importlib.import_module('datasets.dataloader')
        #print("try 문")

    # Data Load
    print("Set up data loader")
    trainLoader, valLoader = DataLoader.create(opt)

    # Check Points Loader
    print('=> Checking checkpoints')
    checkpoint = checkpoints.load(opt)

    # Create model
    model, optimState = models.setup(opt, checkpoint)
    model.cuda()

    criterion = criterions.setup(opt, checkpoint, model)

    trainer = Trainer.createTrainer(model, criterion, opt, optimState)

    if opt.testOnly:
        loss = trainer.test(valLoader, 0)
        sys.exit()


    bestLoss = math.inf
    startEpoch = max([1, opt.epochNum])
    #print("opt.epochNum : ", opt.epochNum)

    if checkpoint != None:
        startEpoch = checkpoint['epoch'] + 1
        bestLoss = checkpoint['loss']
        print('Previous loss: \033[1;36m%1.4f\033[0m' % bestLoss)
#     optimizer.step()
    trainer.LRDecay(startEpoch)
    opt.nEpochs + 1
    print("opt.nEpochs : ",opt.nEpochs)
    # training
    print("Training Start")
    for epoch in range(startEpoch, opt.nEpochs + 1):
        trainer.scheduler.step()

        trainLoss = trainer.train(trainLoader, epoch)
        testLoss = trainer.test(valLoader, epoch)
        
        bestModel = False
        if testLoss < bestLoss:
            bestModel = True
            bestLoss = testLoss
            print(' * Best model: \033[1;36m%1.4f\033[0m * ' % testLoss)

        checkpoints.save(epoch, trainer.model, criterion, trainer.optimizer, bestModel, testLoss ,opt)

    print(' * Finished Err: \033[1;36m%1.4f\033[0m * ' % bestLoss)


if __name__ == '__main__':
    main()
