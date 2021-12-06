#!/usr/bin/env python
# coding=utf-8
'''
Author: Jianheng Liu
Date: 2021-12-04 01:22:13
LastEditors: Jianheng Liu
LastEditTime: 2021-12-04 02:30:45
Description: Description
'''

import torch
from torch._C import dtype
import torch.nn as nn
from torch.nn.functional import mse_loss, relu
from torch.nn.modules.loss import MSELoss
import torch.optim as opt
import torch.functional as F
import numpy as np

class myMLP(nn.Module):
    def __init__(self):
        super(myMLP, self).__init__()
        self.fc1 = nn.Linear(1, 10)
        self.fc2 = nn.Linear(10, 10)
        self.fc3 = nn.Linear(10, 1)
    
    def forward(self, x):
        x = self.fc1(x)
        x = relu(x)
        x = self.fc2(x)
        x = relu(x)
        x = self.fc3(x)

        return x

def train():
    x = np.linspace(0, 1000, 1000)
    y = 2*x + 3
    data = np.stack((x, y), axis=-1).reshape([1000, 1, 2])
    data_batch = torch.tensor(data, dtype=torch.float32)
    print(data_batch.shape)

    model = myMLP()
    loss = MSELoss()
    optimizer = opt.SGD(model.parameters(), 0.01, 0.9)
    model.train()
    for epoch in range(1, 100):
        for i in range(data_batch.shape[0]):
            y_true = data_batch[i][:, 1]
            y_pred = model(data_batch[i][:, 0])

            optimizer.zero_grad()
            loss(y_pred, y_true).backward()
            optimizer.step()

        print('epoch', epoch)
    model.train(False)
    predict = model(data_batch[1][:, 0]*0.5)
    print(predict)

if __name__ == '__main__':
    train()
            
            

            


