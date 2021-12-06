#!/usr/bin/env python
# coding=utf-8
'''
Author: Jianheng Liu
Date: 2021-10-23 23:05:43
LastEditors: Jianheng Liu
LastEditTime: 2021-12-07 00:10:25
Description: MMSegmentor
'''
import torch
from logging import debug
import mmseg
from mmcv.ops import get_compiling_cuda_version, get_compiler_version
from mmseg.apis import inference_segmentor, init_segmentor
import sys
print('sys: ', sys.executable)
print('sys: ', sys.path)

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
