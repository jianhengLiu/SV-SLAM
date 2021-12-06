/*
 * @Author: Jianheng Liu
 * @Date: 2021-10-24 16:46:12
 * @LastEditors: Jianheng Liu
 * @LastEditTime: 2021-11-24 13:41:22
 * @Description: Description
 */
#pragma once

#include "camera/camera.h"
#include "gps/gps.h"
#include "lidar/lidar.h"

#include <eigen3/Eigen/Dense>

struct SensorParam {
  //    Eigen::Matrix4d R_lidar2cam;
  //    Eigen::Vector3d t_lidar2cam;

  Eigen::Matrix4d Tr_lidar2cam;
};

class Sensor {
public:
  Sensor(const std::string &sensor_config_file);

  std::shared_ptr<GPS> gps;
  std::vector<Camera> camera;
  std::shared_ptr<Lidar> lidar;

private:
  std::thread sensor_thread_;

  SensorParam sensor_param_;
  int CAM_NUM;

  void sensorProcess();

  void readParameters(const std::string &sensor_config_file);

  void alignLidar2Img(const pcl::PointCloud<pcl::PointXYZ> &_cloud_in,
                      cv::Mat _image_in);
  cv::Mat
  transformLidar2CamFrame(const pcl::PointCloud<pcl::PointXYZ> &cloud_lidar,
                          pcl::PointCloud<pcl::PointXYZ> &cloud_cam);
};