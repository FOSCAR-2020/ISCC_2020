import os
import cv2
import numpy as np
import time
from torch.autograd import Variable
import torch.optim as optim
import util.visualize as vis
from util.progbar import progbar
import torch
import matplotlib.pyplot as plt

class stackHourglassTrainer():
    def __init__(self, model, criterion, opt, optimState):
        self.model = model
        self.criterion = criterion
        self.optimState = optimState
        self.opt = opt

        if opt.optimizer == 'SGD':
            self.optimizer = optim.SGD(model.parameters(), lr=opt.LR, momentum=opt.momentum, dampening=opt.dampening, weight_decay=opt.weightDecay)
        elif opt.optimizer == 'Adam':
            self.optimizer = optim.Adam(model.parameters(), lr=opt.LR, betas=(opt.momentum, 0.999), eps=1e-8, weight_decay=opt.weightDecay)

        if self.optimState is not None:
            self.optimizer.load_state_dict(self.optimState)

        self.logger = {'train' : open(os.path.join(opt.resume, 'train.log'), 'a+'),
                       'val' : open(os.path.join(opt.resume, 'test.log'), 'a+')}

    def train(self, trainLoader, epoch):
        self.model.train()

        print('=> Training epoch # ' + str(epoch))

        # define avgLoss to Zero
        avgLoss = 0
        visImg = []

        # program 진행 상태를 보여주는 클래스
        self.progbar = progbar(len(trainLoader), width=self.opt.barwidth)

        # trainLoader 에서 데이터 추출 -> torch DataLoader 좀 더 공부..
        for i, (inputData, line, imgids) in enumerate(trainLoader):
            if self.opt.debug and i > 10:
                break

            start = time.time()
            
            #line = line.type(torch.LongTensor)

            # print(inputData.shape)
            print(line.type())
            
            # Tensor 를 Variable Class로 변환 -> 자동으로 변화도를 계산할 수 있음.
            inputData_var, line_var = Variable(inputData), Variable(line)
            self.optimizer.zero_grad()

            if self.opt.GPU:
                inputData_var = inputData_var.cuda()
                line_var = line_var.cuda()

            dataTime = time.time() - start

            # foward 진행    input : original img data, line img data
            # self.model : stackedHGB
            loss, line_loss, line_result = self.model.forward(inputData_var, line_var)

            # backward 진행
            loss.backward()
            self.optimizer.step()

            # run time 계산
            runTime = time.time() - start

            # warning zone : 수정 필요
            avgLoss = (avgLoss * i + loss.data) / (i + 1)

            # warning zone : 수정 필요
            log = 'Epoch: [%d][%d/%d] Time %1.3f Data %1.3f Err %1.4f\n' % (epoch, i, len(trainLoader), runTime, dataTime, loss.data)

            self.logger['train'].write(log)

            # error zone : probar code 수정 필요 -> type Error & moudle Error but learning에는 지장 없음.
            self.progbar.update(i, [('Time', runTime), ('Loss', loss.data)])

            if i <= self.opt.visTrain:
                visImg.append(inputData)
                visImg.append(line_result.cpu().data)
                visImg.append(line)

            #if i == self.opt.visTrain:
            #    self.visualize(visImg, epoch, 'train', trainLoader.dataset.postprocess, trainLoader.dataset.postprocessLine)

        log = '\n * Finished training epoch # %d     Loss: %1.4f\n' % (epoch, avgLoss)
        self.logger['train'].write(log)
        print(log)

        return avgLoss

    def test(self, valLoader, epoch):
        print("*********************************test****************************")
        self.model.eval()

        avgLoss = 0
        visImg = []

        self.progbar = progbar(len(valLoader), width=self.opt.barwidth)

        #print("valLoader : ",len(valLoader))

        for i, (inputData, line, imgids) in enumerate(valLoader):
            if self.opt.debug and i > 10:
                break

            start = time.time()
            with torch.no_grad():
                inputData_var, line_var = Variable(inputData), Variable(line)
            if self.opt.GPU:
                inputData_var = inputData_var.cuda()
                line_var = line_var.cuda()
            dataTime = time.time() - start

            loss, line_loss, line_result = self.model.forward(inputData_var, line_var)

            #print("x_1 shape[0]", x_1.shape)

            #plt.figure(figsize=(8, 8))

            #arr_img1 = []

#             if i == 1:
#                 idx = 1
#                 for j in range(36):
#                     img = x_1[1][j].cpu().detach().numpy()
#                     ax = plt.subplot(6, 6, idx)
#                     ax.set_xticks([])
#                     ax.set_yticks([])
#                     plt.imshow(img, cmap='gray')
#                     idx += 1
#                     # cv2.imshow("img",img)
#                     # cv2.waitKey(300)
#                 plt.show()
            # for _ in range(8):
            #     for _ in range(8):
            #         ax = plt.subplot(8, 8, idx)
            #         ax.set_xticks([])
            #         ax.set_yticks([])
            #         plt.imshow(arr_img1[idx], cmap='gray')
            #         idx += 1


            # print("x_1 shape[0]", x_1[0][0].shape)
            # print("x_1[0][0] type : ", type(x_1[0][0]))
            # img = x_1[0][0].cpu().detach().numpy()
            # print(img)
            # cv2.imshow("img",img)
            # cv2.waitKey(300000)

            runTime = time.time() - start

            avgLoss = (avgLoss * i + loss.data) / (i + 1)

            log = 'Epoch: [%d][%d/%d] Time %1.3f Data %1.3f Err %1.4f\n' % (epoch, i, len(valLoader), runTime, dataTime, loss.data)
            self.logger['val'].write(log)
            self.progbar.update(i, [('Time', runTime), ('Loss', loss.data)])

            if i <= self.opt.visTest:
                visImg.append(inputData.cpu())
                visImg.append(line_result.cpu().data)
                visImg.append(line)

            if i == self.opt.visTest:
                self.visualize(visImg, epoch, 'test', valLoader.dataset.postprocess, valLoader.dataset.postprocessLine)

            outDir = os.path.join(self.opt.resume, str(epoch))
            if not os.path.exists(outDir):
                os.makedirs(outDir)

            for j in range(len(imgids)):
                np.save(os.path.join(outDir, imgids[j] + '_line.npy'), valLoader.dataset.postprocessLine()(line_result.cpu().data[j].numpy()))

        log = '\n * Finished testing epoch # %d      Loss: %1.4f\n' % (epoch, avgLoss)
        self.logger['val'].write(log)
        print(log)

        return avgLoss

    def LRDecay(self, epoch):
        self.scheduler = optim.lr_scheduler.StepLR(self.optimizer, step_size=self.opt.LRDParam, gamma=0.1, last_epoch=epoch-2)

    def LRDecayStep(self):
        self.scheduler.step()

    def visualize(self, visImg, epoch, split, postprocess, postprocessLine):
        outputImgs = []
        for i in range(len(visImg) // 3):
            for j in range(self.opt.batchSize):
                outputImgs.append(postprocess()(visImg[3 * i][j].numpy()))
                outputImgs.append(postprocessLine()(visImg[3 * i + 1][j].numpy()))
                outputImgs.append(postprocessLine()(visImg[3 * i + 2][j].numpy()))
        vis.writeImgHTML(outputImgs, epoch, split, 3, self.opt)

    def visJunc(self, img, junc, opt):
        juncConf, juncRes, juncBinConf, juncBinRes = junc
        juncConf, juncRes, juncBinConf, juncBinRes = juncConf.numpy(), juncRes.numpy(), juncBinConf.numpy(), juncBinRes.numpy()
        imgDim = opt.imgDim
        thres = opt.visThres
        out = img.astype(np.uint8).copy()
        blockSize = 8
        for i in range(imgDim // blockSize):
            for j in range(imgDim // blockSize):
                if juncConf[i][j] > thres:
                    p = np.array((i, j)) * np.array((blockSize, blockSize))
                    p = p + (juncRes[:, i, j] + 0.5) * np.array((blockSize, blockSize))
                    for t in range(12):
                        if juncBinConf[t, i, j] > thres:
                            theta = (t + juncBinRes[t, i, j] + 0.5) * 30
                            theta = theta * np.pi / 180
                            end = p + 20 * np.array((np.cos(theta), np.sin(theta)))
                            out = cv2.line(out, self.regInt(p), self.regInt(end), (0, 255, 0), 2)
        return out

    def regInt(self, x):
        return (int(round(x[0])), int(round(x[1])))

def createTrainer(model, criterion, opt, optimState):
    return stackHourglassTrainer(model, criterion, opt, optimState)
