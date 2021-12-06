#!/usr/bin/env python
# coding=utf-8
'''
Author: Jianheng Liu
Date: 2021-12-04 20:40:48
LastEditors: Jianheng Liu
LastEditTime: 2021-12-04 20:41:57
Description: Description
'''
import onnxruntime as ort
import numpy as np
ort_session = ort.InferenceSession('/home/chrisliu/ROSws/SV-SLAM_ws/src/SV-SLAM/models/ddrnet23_slim.onnx')
outputs = ort_session.run(None,{'input':np.random.randn(10,20),'input_mask':np.random.randn(1,20,5)})
# 由于设置了dynamic_axes,支持对应维度的变化
outputs = ort_session.run(None,{'input':np.random.randn(10,5),'input_mask':np.random.randn(1,26,2)})
# outputs 为 包含'output'和'output_mask'的list

import onnx
model = onnx.load('/home/chrisliu/ROSws/SV-SLAM_ws/src/SV-SLAM/models/ddrnet23_slim.onnx')
onnx.checker.check_model(model)
