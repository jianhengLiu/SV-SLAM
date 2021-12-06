#!/usr/bin/env python
# coding=utf-8
'''
Author: Jianheng Liu
Date: 2021-12-05 00:08:33
LastEditors: Jianheng Liu
LastEditTime: 2021-12-05 12:48:27
Description: Description
'''
import torch
import torch.nn as nn
from collections import OrderedDict
# An instance of your model.
from DDRNet_23_slim import DualResNet,BasicBlock
import torchvision.transforms as transforms
model = DualResNet(BasicBlock, [2, 2, 2, 2], num_classes=19, planes=32, spp_planes=128, head_planes=64, augment=False)

# model = nn.DataParallel(model)
# cudnn.benchmark = True
# model = nn.DataParallel(model).cuda()

# checkpoint = torch.load('/home/chrisliu/Gitresposities/DDRNet.pytorch/pretrained_models/best_val_smaller.pth', map_location='cpu') 
   
# new_state_dict = OrderedDict()
# for k, v in checkpoint['state_dict'].items():
#     name = k[7:]  
#     new_state_dict[name] = v
#         #model_dict.update(new_state_dict)
#         #model.load_state_dict(model_dict)

# model.load_state_dict(new_state_dict, strict = False)

model.load_state_dict(torch.load('/home/chrisliu/Gitresposities/DDRNet.pytorch/pretrained_models/best_val_smaller.pth'), strict = False)
model.cuda()
model.eval()
 
# An example input you would normally provide to your model's forward() method.
testsize = 224
example = torch.rand(1, 3, 1024, 1024)
 
example = example.cuda()
# Use torch.jit.trace to generate a torch.jit.ScriptModule via tracing.
traced_script_module = torch.jit.trace(model, example)
print(traced_script_module.code)
traced_script_module.save("/home/chrisliu/ROSws/SV-SLAM_ws/src/SV-SLAM/models/best_val_smaller.pt")