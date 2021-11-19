/*
 * @Author: Jianheng Liu
 * @Date: 2021-10-31 23:14:47
 * @LastEditors: Jianheng Liu
 * @LastEditTime: 2021-11-17 22:15:02
 * @Description: Description
 */
#include "segmentor.h"

#include <stdio.h>

#include <pybind11/embed.h> // everything needed for embedding
#include <pybind11/numpy.h>
namespace py = pybind11;

using namespace std;

std::shared_ptr<py::scoped_interpreter> py_interpreter;

py::module_ sys;
py::object mmsegmentor;

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

  py_interpreter =
      std::make_shared<py::scoped_interpreter>(); // start the interpreter and keep it alive

   sys = py::module_::import("sys");
   py::print(sys.attr("executable"));

  mmsegmentor = py::module::import("segmentor.mmsegmentor").attr("Segmentor")(CONFIG_PATH, CHECKPOINT_PATH, DEVICE);
};

void Segmentor::readParameters(const std::string &sensor_config_file)
{
  cout << sensor_config_file << endl;
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
  py::array_t<unsigned char> dst = py::array_t<unsigned char>({input_img.rows, input_img.cols, 3}, input_img.data);

  py::array_t<unsigned char> result = mmsegmentor.attr("infer")(dst);

  // https://blog.csdn.net/qinlele1994/article/details/108345477
  py::buffer_info buf = result.request();
  cv::Mat encode_img(buf.shape[1], buf.shape[2], CV_8UC1, (unsigned char *)buf.ptr);

  if (SHOW_IMG)
  {
    cv::Mat debug_img = cv::Mat::zeros(buf.shape[1], buf.shape[2], CV_8UC3);
    for (int i = 0; i < encode_img.rows; ++i)
    {
      for (int k = 0; k < encode_img.cols; ++k)
      {
        cv::circle(debug_img, cv::Point(k, i), 1, labels[encode_img.at<uchar>(i, k)].color);
      }
    }
    debug_img = 0.5 * debug_img + 0.5 * input_img;
    cv::imshow("debug_img", debug_img);
    cv::waitKey(1);
  }
  return encode_img;
}