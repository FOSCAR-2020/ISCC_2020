import torch
import torch.nn as nn
import torch.nn.functional as F

class DoubleConv(nn.Module):
    def __init__(self, in_channels, out_channels, mid_channels=None):
        super().__init__()
        
        if not mid_channels:
            mid_channels = out_channels
        
        self.double_conv = nn.Sequential(
            nn.Conv2d(in_channels, mid_channels, kernel_size=3, padding=1),
            nn.BatchNorm2d(mid_channels),
            nn.ReLU(inplace=True),
            nn.Conv2d(mid_channels, out_channels, kernel_size=3, padding=1),
            nn.BatchNorm2d(out_channels),
            nn.ReLU(inplace=True)
        )
    
    def forward(self, x):
        return self.double_conv(x)


class Down(nn.Module):
    def __init__(self, in_channels, out_channels):
        super().__init__()
        self.maxpool_conv = nn.Sequential(
            nn.MaxPool2d(2),
            DoubleConv(in_channels, out_channels)
        )
    
    def forward(self, x):
        return self.maxpool_conv(x)

class Up(nn.Module):
    def __init__(self, in_channels, out_channels, bilinear=True):
        super().__init__()
        
        if bilinear :
            self.up = nn.Upsample(scale_factor=2, mode='bilinear', align_corners=True)
            self.conv = DoubleConv(in_channels, out_channels, in_channels // 2)
        else:
            self.up = nn.ConvTranspose2d(in_channels, in_channels//2, kernel_size = 2, stride = 2)
            self.conv = DoubleConv(in_channels, out_channels)
        
    
    def forward(self, x1, x2):
        x1 = self.up(x1)
        
        diffY = x2.size()[2] - x1.size()[2]
        diffX = x2.size()[3] - x1.size()[3]
        
        # padding
        x1 = F.pad(x1, [diffX // 2, diffX - diffX //2,
                       diffY // 2, diffY - diffY // 2])
        
        x = torch.cat([x2, x1], dim = 1)
        return self.conv(x)

class OutConv(nn.Module):
    def __init__(self, in_channels, out_channels):
        super(OutConv, self).__init__()
        self.conv = nn.Conv2d(in_channels, out_channels, kernel_size = 1)
        
    def forward(self, x):
        return self.conv(x)
    

class Unet(nn.Module):
    def __init__(self, n_channels, n_classes, bilinear = True):
        super(Unet, self).__init__()
        
        self.n_channels = n_channels
        self.n_classes = n_classes
        self.bilinear = bilinear
        
        # conv2d -> batch -> relu -> conv2d -> batch -> relu
        self.inc = DoubleConv(n_channels, 64)
        self.down1 = Down(64, 128)
        self.down2 = Down(128, 256)
        self.down3 = Down(256, 512)
        
        factor = 2 if bilinear else 1
        
        self.down4 = Down(512, 1024 // factor)
        
        self.up1 = Up(1024, 512 // factor, bilinear)
        self.up2 = Up(512, 256 // factor, bilinear)
        self.up3 = Up(256, 128 // factor, bilinear)
        self.up4 = Up(128, 64, bilinear)
        self.outc = OutConv(64, n_classes)
    
    def forward(self, x):
        #print("x.shape : ", x.shape)
        x1 = self.inc(x)
        #print("x1.shape : ", x1.shape)
        x2 = self.down1(x1)
        #print("x2.shape : ", x2.shape)
        x3 = self.down2(x2)
        #print("x3.shape : ", x3.shape)
        x4 = self.down3(x3)
        #print("x4.shape : ", x4.shape)
        x5 = self.down4(x4)
        #print("x5.shape : ", x5.shape)
        x = self.up1(x5, x4)
        #print("x.shape : ", x.shape)
        x = self.up2(x, x3)
        #print("x.shape : ", x.shape)
        x = self.up3(x, x2)
        #print("x.shape : ", x.shape)
        x = self.up4(x, x1)
        #print("x.shape : ", x.shape)
        logits = self.outc(x)
        #print("logits.shape : ", logits.shape)
        
        return logits
    
    
def createModel(opt):
    model = Unet(n_channels = 3, n_classes = 1, bilinear=True)

    return model
        
        