/*
 * @Author: Jianheng Liu
 * @Date: 2021-10-31 23:14:47
 * @LastEditors: Jianheng Liu
 * @LastEditTime: 2021-12-05 12:58:44
 * @Description: Description
 */

#include "segmentor.h"

#include <stdio.h>

#include <pybind11/embed.h> // everything needed for embedding
#include <pybind11/numpy.h>
namespace py = pybind11;

#include <opencv2/dnn/dnn.hpp>

using namespace std;

std::shared_ptr<py::scoped_interpreter> py_interpreter;

py::module_ sys;
py::object mmsegmentor;

#include <torch/script.h>
#include <torch/torch.h>

torch::jit::script::Module module;
torch::Device device(torch::kCUDA);

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

//上色
void Visualization(cv::Mat prediction_map, std::string LUT_file)
{

  // cv::cvtColor(prediction_map.clone(), prediction_map, cv::COLOR_GRAY2BGR);
  // cv::Mat label_colours = cv::imread(LUT_file, 1);
  // cv::cvtColor(label_colours, label_colours, cv::COLOR_RGB2BGR);
  cv::Mat output_image(prediction_map.rows, prediction_map.cols, CV_8UC3);
  // cv::LUT(prediction_map, label_colours, output_image);
  for (int i = 0; i < prediction_map.rows; ++i)
    for (int k = 0; k < prediction_map.cols; ++k)
    {
      cv::circle(output_image, cv::Point(k, i), 1, labels[prediction_map.at<uchar>(i, k)].color);
    }

  cv::imshow("Display window", output_image);
  cv::waitKey(1);
}

Segmentor::Segmentor(const std::string &sensor_config_file)
{
  readParameters(sensor_config_file);

  // net = cv::dnn::readNet("/home/chrisliu/ROSws/SV-SLAM_ws/src/SV-SLAM/models/ddrnet23_slim.onnx");

  // py_interpreter =
  //     std::make_shared<py::scoped_interpreter>(); // start the interpreter and keep it alive

  //  sys = py::module_::import("sys");
  //  py::print(sys.attr("executable"));
  //  py::print(sys.attr("path"));

  // mmsegmentor = py::module::import("segmentor.mmsegmentor").attr("Segmentor")(CONFIG_PATH, CHECKPOINT_PATH, DEVICE);

  // torch::DeviceType device_type;
  // if (torch::cuda::is_available())
  // {
  //   std::cout << "CUDA available! Predicting on GPU." << std::endl;
  //   device_type = torch::kCUDA;
  // }
  // else
  // {
  //   std::cout << "Predicting on CPU." << std::endl;
  //   device_type = torch::kCPU;
  // }
  // device = torch::Device(device_type);

  //Init model
  std::string model_pb = "/home/chrisliu/ROSws/SV-SLAM_ws/src/SV-SLAM/models/best_val_smaller.pt";
  module = torch::jit::load(model_pb);
  module.to(torch::kCUDA);
  module.eval();
  // std::cout<<module<<std::endl;
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

  fn["show_img"] >> SHOW_IMG;

  fsSettings.release();
}

cv::Mat Segmentor::infer(const cv::Mat &input_img)
{
  // auto image = cv::imread("/home/chrisliu/Datasets/kitti/2011_09_26_drive_0101_sync/2011_09_26/2011_09_26_drive_0101_sync/image_02/data/0000000000.png", cv::ImreadModes::IMREAD_COLOR);
  // cv::cvtColor(image, image, cv::COLOR_BGR2RGB);// bgr -> rgb

  cv::Mat img_float;
  cv::resize(input_img, img_float, cv::Size(1024, 1024));
  cv::imshow("img_float", img_float);
  cv::waitKey(1);

  torch::Tensor img_var = torch::from_blob(img_float.data, {1, 1024, 1024, 3}, torch::kByte).to(device); //将图像转化成张量
  img_var = img_var.permute({0, 3, 1, 2});                                                              //将张量的参数顺序转化为 torch输入的格式 1,3,224,224
  img_var = img_var.toType(torch::kFloat);
  img_var = img_var.div(255);

  std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
  torch::Tensor result = module.forward({img_var}).toTensor();
  std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
  std::cout << "Processing time = " << (std::chrono::duration_cast<std::chrono::microseconds>(t2 - t1).count()) / 1000000.0 << " sec" << std::endl;
  result = result.argmax(1);      //找出每个点概率最大的一个
  result = result.squeeze();      //删除一个维度
  result = result.to(torch::kU8); //.mul(100); //这里是为了让分割的区域更明显,但是不需要加，因为后面使用了lut的方法可以使不同的mask显示不同颜色
  result = result.to(torch::kCPU);
  // std::cout << result << std::endl;
  std::cout << result.sizes() << std::endl;
  cv::Mat pts_mat(cv::Size(result.size(0), result.size(1)), CV_8U, result.data_ptr());     //新建一个矩阵，用于保存数据，将tensor的数据转移到这里面
  Visualization(pts_mat, "/home/chrisliu/ROSws/SV-SLAM_ws/src/SV-SLAM/models/pascal.png"); //上色
  // cv::imshow("output", pts_mat);
  // auto max_result = output.max(1, true);
  // auto max_index = std::get<1>(max_result).item<float>();
  // std::cout << result << std::endl;

  /* cv::Mat blob;
  cv::dnn::blobFromImage(input_img, blob);

  // inference and time
  double t = cv::getTickCount();
  net.setInput(blob);
  cv::Mat output = net.forward();
  t = (cv::getTickCount() - t) / cv::getTickFrequency();
  std::cout << "Output shape: " << output.size() << ", Time-cost: " << t << std::endl;

  cv::imshow("output", output);
  cv::waitKey(1); */

  // py::array_t<unsigned char> dst = py::array_t<unsigned char>({input_img.rows, input_img.cols, 3}, input_img.data);

  // py::array_t<unsigned char> result = mmsegmentor.attr("infer")(dst);

  // // https://blog.csdn.net/qinlele1994/article/details/108345477
  // py::buffer_info buf = result.request();
  // cv::Mat encode_img(buf.shape[1], buf.shape[2], CV_8UC1, (unsigned char *)buf.ptr);

  // if (SHOW_IMG)
  // {
  //   cv::Mat debug_img = cv::Mat::zeros(buf.shape[1], buf.shape[2], CV_8UC3);
  //   for (int i = 0; i < encode_img.rows; ++i)
  //   {
  //     for (int k = 0; k < encode_img.cols; ++k)
  //     {
  //       cv::circle(debug_img, cv::Point(k, i), 1, labels[encode_img.at<uchar>(i, k)].color);
  //     }
  //   }
  //   debug_img = 0.5 * debug_img + 0.5 * input_img;
  //   cv::imshow("debug_img", debug_img);
  //   cv::waitKey(1);
  // }
  // return encode_img;
}