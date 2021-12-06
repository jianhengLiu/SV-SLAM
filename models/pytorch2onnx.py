import torch
import torch.onnx
from DDRNet_23_slim import DualResNet,BasicBlock
import os

def pth_to_onnx(input, checkpoint, onnx_path, input_names=['input'], output_names=['output'], device='cpu'):
    if not onnx_path.endswith('.onnx'):
        print('Warning! The onnx model name is not correct,\
              please give a name that ends with \'.onnx\'!')
        return 0

    model = DualResNet(BasicBlock, [2, 2, 2, 2], num_classes=19, planes=32, spp_planes=128, head_planes=64) #导入模型
    model.load_state_dict(torch.load(checkpoint)) #初始化权重
    model.eval()
    # model.to(device)
    
    torch.onnx.export(model, input, onnx_path, verbose=True, input_names=input_names, output_names=output_names) #指定模型的输入，以及onnx的输出路径
    print("Exporting .pth model to onnx model has been successful!")

if __name__ == '__main__':
    os.environ['CUDA_VISIBLE_DEVICES']='2'
    checkpoint = './best_val_smaller.pth'
    onnx_path = './best_val_smaller.onnx'
    input = torch.randn(1, 3, 1024, 2048)
    # device = torch.device("cuda:2" if torch.cuda.is_available() else 'cpu')
    pth_to_onnx(input, checkpoint, onnx_path)

