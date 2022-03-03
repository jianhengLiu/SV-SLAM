/*
 * @Author: Jianheng Liu
 * @Date: 2021-10-24 16:43:34
 * @LastEditors: Jianheng Liu
 * @LastEditTime: 2021-12-07 00:09:59
 * @Description: Description
 */
#pragma once

#include <condition_variable>
#include <mutex>
#include <queue>
#include <stdio.h>
#include <thread>

#include <eigen3/Eigen/Dense>

#include <sensor_msgs/Image.h>

#include "segmentor/segmentor.h"

struct CamParam {
  std::string model_type;
  int width;
  int height;

  Eigen::Matrix4d R_rect = Eigen::Matrix4d::Identity();
  Eigen::Matrix<double, 3, 4> Proj;

  cv::Mat intrinsic_matrix;
  cv::Mat distortion_coeffs;
  cv::Mat map1, map2;
};

class Camera {
public:
  Camera(const int cam_id, const std::string &sensor_config_file);

  void input_img(const sensor_msgs::ImageConstPtr &_img_msg);


  cv::Mat undistort(cv::Mat &_img_in);

  sensor_msgs::ImageConstPtr getpopCamMsg();

  CamParam cam_param_;
  int CAM_ID;
  std::string IMAGE_TOPIC;

private:
  std::shared_ptr<Segmentor> segmentor;

  std::string sensor_config_file_;

  bool ENABLE_SEGMENTOR;

  std::queue<sensor_msgs::ImageConstPtr> cam_buf_;
  std::queue<cv::Mat> seg_buf;

  void segmentorProcess();

  void readParameters(const std::string &sensor_config_file);
};