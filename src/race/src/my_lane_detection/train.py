import os
import opts
import math
import importlib
import _init_paths

import torch
import torchvision.transforms as transforms
from torch.autograd import Variable

from PIL import Image
import cv2
import numpy as np

def main():
    os.environ["CUDA_VISIBLE_DEVICES"]= '0'
    
    # OPTION PARSER
    opt = opts.parse()
    
    # Device ID
    print("Device ID : {}".format(torch.cuda.current_device()))
    # torch.version : 1.4.0
    print("torch.version : ", torch.__version__)
    # cuda version : 10.1
    print("cuda_version : ", torch.version.cuda)
    
    # model init
    models = importlib.import_module('models.init')
    # criterions init
    criterions = importlib.import_module('criterions.init')
    # checkpoints init
    checkpoints = importlib.import_module('checkpoints')
    # Trainer init
    Trainer = importlib.import_module('models.' + opt.netType + '-train')
    
    # DataLoader Init
    DataLoader = importlib.import_module('datasets.dataloader')
    
    # Data Load
    trainLoader, valLoader = DataLoader.create(opt)
    
    # Check Points Loader
    print("=> Checking checkpoints")
    checkpoint = checkpoints.load(opt)
    
    # Create model
    model, optimState = models.setup(opt, checkpoint)
    model.cuda()
    
    criterion = criterions.setup(opt, checkpoint, model)
    
    trainer = Trainer.createTrainer(model, criterion, opt, optimState)
    
    bestLoss = math.inf
    startEpoch = max([1, opt.epochNum])
    
    if checkpoint != None:
        startEpoch = checkpoint['epoch'] + 1
        bestLoss = checkpoint['loss']
        print('Previous loss: \033[1;36m%1.4f\033[0m' % bestLoss)
    
    trainer.LRDecay(startEpoch)
    
    print("***** Training Start *****")
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