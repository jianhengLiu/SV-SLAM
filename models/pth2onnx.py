#!/usr/bin/env python
# coding=utf-8
'''
Author: Jianheng Liu
Date: 2021-12-04 15:28:09
LastEditors: Jianheng Liu
LastEditTime: 2021-12-04 16:11:36
Description: Description
'''
import torch

device = torch.device("cuda" if torch.cuda.is_available() else "cpu")

dummy_input = torch.randn(1, 3, 224, 224, device='cuda')
model = torch.load("/home/chrisliu/Gitresposities/semantic-segmentation/ddrnet_23slim_city.pth")

# model.eval()

# //给输入输出取个名字
input_names = ["input_1"]
output_names = ["output_1"]

torch.onnx.export(model, dummy_input, "ddrnet_23slim_city.onnx", verbose=True, input_names=input_names, output_names=output_names)
