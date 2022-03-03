#!/usr/bin/env python
# coding=utf-8
'''
Author: Jianheng Liu
Date: 2021-10-23 23:05:43
LastEditors: Jianheng Liu
LastEditTime: 2021-12-03 11:15:43
Description: MMSegmentor
'''

# https://www.cnblogs.com/noluye/p/11446146.html
import argparse

import os
import cv2
import numpy as np
# Check Pytorch installation


import sys
print('sys: ', sys.executable)

from mmseg.apis import inference_segmentor, init_segmentor, show_result_pyplot
from mmseg.core.evaluation import get_palette
from mmcv.ops import get_compiling_cuda_version, get_compiler_version
import mmseg
from logging import debug
import torch
print('torch: ', torch.__version__, torch.cuda.is_available())


# Check MMSegmentation installation
print('mmseg: ', mmseg.__version__)

# Check mmcv installation
print('cuda: ', get_compiling_cuda_version())
print('compiler: ', get_compiler_version())


class Segmentor:

    def __init__(self, config_path, checkpoint_path, device):
        # # Choose to use a config and initialize the detecto
        # Config file
        self.config_path = config_path
        # Checkpoint file
        self.checkpoint_path = checkpoint_path
        # Device used for inference
        self.device = device

        print('config_path:', self.config_path)
        print('checkpoint_path:', self.checkpoint_path)
        print('device:', self.device)
        # build the model from a config file and a checkpoint file
        self.model = init_segmentor(
            self.config_path, self.checkpoint_path, device=self.device)

        if self.model is not None:
            print('Segmentor is Initialized!')

    def infer(self, data):
        if data is not None:
            return inference_segmentor(self.model, data)


if __name__ == '__main__':
    segmentor = Segmentor('/home/chrisliu/ROSws/SV-SLAM_ws/src/SV-SLAM/lib/mmsegmentation/configs/bisenetv1/bisenetv1_r18-d32_4x4_1024x1024_160k_cityscapes.py',
                    '/home/chrisliu/ROSws/SV-SLAM_ws/src/SV-SLAM/lib/mmsegmentation/checkpoints/bisenetv1_r18-d32_4x4_1024x1024_160k_cityscapes_20210922_172239-c55e78e2.pth', 'cuda:0')

    path = '/home/chrisliu/Datasets/kitti/2011_09_26_drive_0101_sync/2011_09_26/2011_09_26_drive_0101_sync/image_02/data'
    path_encode_output = path +'/'+'seg_encode_results/'
    path_img_output = path +'/'+'seg_img_results/'
    path_list = os.listdir(path)
    path_list.sort()

    palette = get_palette('cityscapes')
    palette = np.array(palette)
    opacity = 0.5
    # print(path_list)
    for img_name in path_list:
        img_path = path+'/'+img_name
        result = segmentor.infer(img_path)

        seg_result = result[0]
        seg_img = np.zeros((seg_result.shape[0], seg_result.shape[1], 1), dtype=np.uint8)
        color_img = np.zeros((seg_result.shape[0], seg_result.shape[1], 3), dtype=np.uint8)
        for label, color in enumerate(palette):
            seg_img[seg_result == label, :] = label
            color_img[seg_result == label, :] = color

        # convert to BGR
        color_img = color_img[..., ::-1]
        img = cv2.imread(img_path)
        color_img = img * (1 - opacity) + color_img * opacity
        color_img = color_img.astype(np.uint8)

        cv2.imwrite(path_encode_output+img_name, seg_img)
        cv2.imwrite(path_img_output+img_name, color_img)

        cv2.imshow('seg_img', seg_img)
        cv2.imshow('color_img', color_img)
        cv2.waitKey(1)
