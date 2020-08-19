import os
import cv2
import numpy as np
import torch
import time
from torch.autograd import Variable
import torch.optim as optim
import util.visualize as vis
from util.progbar import progbar
import matplotlib.pyplot as plt
from torch.utils.tensorboard import SummaryWriter

class UnetTrainer():
    def __init__(self, model, criterion, opt, optimState):
        self.model = model
        self.criterion = criterion
        self.optimState = optimState
        self.opt = opt
        self.n_classes = 1
        
        if opt.optimizer == 'SGD':
            self.optimizer = optim.SGD(model.parameters(), lr=opt.LR, momentum=opt.momentum, dampening=opt.dampening, weight_decay=opt.weightDecay)
        elif opt.optimizer == 'Adam':
            self.optimizer = optim.Adam(model.parameters(), lr=opt.LR, betas=(opt.momentum, 0.999), eps=1e-8, weight_decay = opt.weightDecay)
        
        if self.optimState is not None:
            self.optimizer.load_state_dict(self.optimState)
            
        self.logger = {'train' : open(os.path.join(opt.resume, 'train.log'), 'a+'),
                      'val' : open(os.path.join(opt.resume, 'test.log'), 'a+')}
        
    def train(self, trainLoader, epoch):
        self.model.train()
        
        print("-> Training epoch # {}".format(str(epoch)))
        
        avgLoss = 0
        
        self.progbar = progbar(len(trainLoader), width=self.opt.barwidth)
        
        for i, (inputData, seg, imgids) in enumerate(trainLoader):
            if self.opt.debug and i > 10:
                break
            
            start = time.time()
            
            inputData_var, seg_var = Variable(inputData), Variable(seg)
            
            self.optimizer.zero_grad()
            
            if self.opt.GPU:
                inputData_var = inputData_var.cuda()
                seg_var = seg_var.cuda()
            
            dataTime = time.time() - start
            
            output = self.model.forward(inputData_var)
            
            loss = self.criterion(output, torch.gt(seg_var, 0).type(torch.float32))
            
            loss.backward()
            
            self.optimizer.step()
            
            runTime = time.time() - start
            
            avgLoss = (avgLoss * i + loss.data) / (i + 1)
            
            log = 'Epoch: [%d][%d/%d] Time %1.3f Data %1.3f Err %1.4f\n' % (epoch, i, len(trainLoader), runTime, dataTime, loss.data)
            
            self.logger['train'].write(log)
            
            self.progbar.update(i, [('Time', runTime), ('Loss', loss.data)])
        
        log = '\n * Finished training epoch # %d     Loss: %1.4f\n' % (epoch, avgLoss)
        self.logger['train'].write(log)
        print(log)
        
        return avgLoss
    
    def test(self, valLoader, epoch):
        self.model.eval()
        
        avgLoss = 0
        
        self.progbar = progbar(len(valLoader), width=self.opt.barwidth)
        
        for i, (inputData, seg, imgids) in enumerate(valLoader):
            if self.opt.debug and i > 10:
                break
            
            start = time.time()
            
            with torch.no_grad():
                inputData_var, seg_var = Variable(inputData), Variable(seg)
                
                if self.opt.GPU:
                    inputData_var = inputData_var.cuda()
                    seg_var = seg_var.cuda()
                
                dataTime = time.time() - start
                
                output = self.model.forward(inputData_var)
                
                loss = self.criterion(output, torch.gt(seg_var, 0).type(torch.float32))
                
                runTime = time.time() - start
                
                avgLoss = (avgLoss * i + loss.data) / (i+1)
                
                log = 'Epoch: [%d][%d/%d] Time %1.3f Data %1.3f Err %1.4f\n' % (epoch, i, len(valLoader), runTime, dataTime, loss.data)
                
                self.logger['val'].write(log)
                self.progbar.update(i, [('Time', runTime), ('Loss', loss.data)])
        
        log = '\n * Finished testing epoch # %d      Loss: %1.4f\n' % (epoch, avgLoss)
        self.logger['val'].write(log)
        print(log)
        
        return avgLoss
    
    def LRDecay(self, epoch):
        self.scheduler = optim.lr_scheduler.StepLR(self.optimizer, step_size=self.opt.LRDParam, gamma=0.1, last_epoch=epoch-2)
        
    def LRDecayStep(self):
        self.scheduler.step()

def createTrainer(model, criterion, opt, optimState):
    return UnetTrainer(model, criterion, opt, optimState)
        
        
                
            
            
            
            
        
