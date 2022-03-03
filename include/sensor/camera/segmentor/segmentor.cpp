/*
 * @Author: Jianheng Liu
 * @Date: 2021-10-31 23:14:47
 * @LastEditors: Jianheng Liu
 * @LastEditTime: 2021-12-06 23:33:35
 * @Description: Description
 */

#include "segmentor.h"

#include <stdio.h>

#include <opencv2/dnn/dnn.hpp>

#include <torch/script.h>
#include <torch/torch.h>

torch::jit::script::Module module;
std::shared_ptr<torch::Device> p_device;

using namespace std;

struct Label
{
  /* data */
  std::string name;
  std::string category;
  cv::Scalar color;
};
map<int, Label> labels = {
    // https://github.com/mcordts/cityscapesScripts/blob/master/cityscapesscripts/helpers/labels.py
    //trainId   name    category   color
    {255, {"void", "void", {0, 0, 0}}},
    {0, {"road", "flat", {128, 64, 128}}},
    {1, {"sidewalk", "flat", {232, 35, 244}}},
    {2, {"building", "construction", {70, 70, 70}}},
    {3, {"wall", "construction", {156, 102, 102}}},
    {4, {"fence", "construction", {153, 153, 190}}},
    {5, {"pole", "object", {153, 153, 153}}},
    {6, {"traffic light", "object", {30, 170, 250}}},
    {7, {"traffic sign", "object", {0, 220, 220}}},
    {8, {"vegetation", "nature", {35, 142, 107}}},
    {9, {"terrain", "nature", {152, 251, 152}}},
    {10, {"sky", "sky", {180, 130, 70}}},
    {11, {"person", "human", {60, 20, 220}}},
    {12, {"rider", "human", {0, 0, 255}}},
    {13, {"car", "vehicle", {142, 0, 0}}},
    {14, {"truck", "vehicle", {70, 0, 0}}},
    {15, {"bus", "vehicle", {100, 60, 0}}},
    {16, {"train", "vehicle", {100, 80, 0}}},
    {17, {"motorcycle", "vehicle", {230, 0, 0}}},
    {18, {"bicycle", "vehicle", {32, 11, 119}}},
    {-1, {"license plate", "vehicle", {142, 0, 0}}}};

Segmentor::Segmentor(const std::string &sensor_config_file)
{
  readParameters(sensor_config_file);

  torch::DeviceType device_type;
  if (torch::cuda::is_available())
  {
    std::cout << "CUDA available! Predicting on GPU." << std::endl;
    device_type = torch::kCUDA;
  }
  else
  {
    std::cout << "Predicting on CPU." << std::endl;
    device_type = torch::kCPU;
  }
  p_device = std::make_shared<torch::Device>(device_type);

  //Init model
  
  module = torch::jit::load(TORCH_SCRIPT_PATH);
  module.to(torch::kCUDA);
  module.eval();
  std::cout << "Segmentor is Initialized!" << std::endl;
};

void Segmentor::readParameters(const std::string &sensor_config_file)
{
  // cout << sensor_config_file << endl;
  cv::FileStorage fsSettings(sensor_config_file, cv::FileStorage::READ);
  if (!fsSettings.isOpened())
  {
    std::cerr << "ERROR: Wrong path to settings" << std::endl;
  }

  cv::FileNode fn = fsSettings["segmentor_config"];

  fn["config_path"] >> CONFIG_PATH;
  fn["checkpoint_path"] >> CHECKPOINT_PATH;
  fn["device"] >> DEVICE;

  fn["torch_script_path"] >> TORCH_SCRIPT_PATH;

  fn["show_img"] >> SHOW_IMG;

  fsSettings.release();
}

cv::Mat Segmentor::infer(cv::Mat &input_img)
{

  std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
  cv::Mat img_float;
  cv::resize(input_img, img_float, cv::Size(1024, 512));

  torch::Tensor img_var = torch::from_blob(img_float.data, {1, 512, 1024, 3}, torch::kByte).to(*p_device); //将图像转化成张量
  img_var = img_var.permute({0, 3, 1, 2});                                                                 //将张量的参数顺序转化为 torch输入的格式 1,3,224,224
  img_var = img_var.toType(torch::kFloat);
  img_var = img_var.div(255);

  torch::Tensor result = module.forward({img_var}).toTensor();
  // result = result.argmax(1);      //找出每个点概率最大的一个
  result = result.squeeze();      //删除一个维度
  result = result.to(torch::kU8); //.mul(100); //这里是为了让分割的区域更明显,但是不需要加，因为后面使用了lut的方法可以使不同的mask显示不同颜色
  result = result.to(torch::kCPU);
  // std::cout << result << std::endl;
  // std::cout << result.sizes() << std::endl;
  // std::cout << result.size(0) << std::endl;
  // std::cout << result.size(1) << std::endl;
  cv::Mat seg_encode(cv::Size(result.size(1), result.size(0)), CV_8U, result.data_ptr()); //新建一个矩阵，用于保存数据，将tensor的数据转移到这里面
  cv::resize(seg_encode, seg_encode, cv::Size(input_img.cols, input_img.rows));
  if (SHOW_IMG)
  {
    cv::Mat seg_img = cv::Mat::zeros(seg_encode.rows, seg_encode.cols, CV_8UC3);
    for (int i = 0; i < seg_encode.rows; ++i)
      for (int k = 0; k < seg_encode.cols; ++k)
      {
        cv::circle(seg_img, cv::Point(k, i), 1, labels[seg_encode.at<uchar>(i, k)].color);
      }
    seg_img = 0.5 * seg_img + 0.5 * input_img;
    cv::imshow("Display window", seg_img);
    cv::waitKey(1);
  }

  std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
  std::cout << "Processing time = " << (std::chrono::duration_cast<std::chrono::microseconds>(t2 - t1).count()) / 1000000.0 << " sec" << std::endl;

  return seg_encode;
}