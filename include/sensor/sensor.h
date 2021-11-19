/*
 * @Author: Jianheng Liu
 * @Date: 2021-10-24 16:46:12
 * @LastEditors: Jianheng Liu
 * @LastEditTime: 2021-11-17 22:26:33
 * @Description: Description
 */
#pragma once

#include "gps/gps.h"
#include "image/image.h"
#include "lidar/lidar.h"

#include <eigen3/Eigen/Dense>

struct SensorParam
{
    //    Eigen::Matrix4d R_lidar2cam;
    //    Eigen::Vector3d t_lidar2cam;

    Eigen::Matrix4d Tr_lidar2cam;
};

class Sensor
{
public:
    Sensor(const std::string &sensor_config_file);

    std::shared_ptr<GPS> gps;
    std::shared_ptr<Image> img;
    std::shared_ptr<Lidar> lidar;

private:
    std::thread sensor_thread_;

    SensorParam sensor_param_;

    void sensorProcess();

    void readParameters(const std::string &sensor_config_file);

    void alignLidar2Img(const pcl::PointCloud<pcl::PointXYZ> &cloud_in, pcl::PointCloud<pcl::PointXYZ> &cloud_out);
    cv::Mat transformLidar2CamFrame(const pcl::PointCloud<pcl::PointXYZ> &cloud_lidar, pcl::PointCloud<pcl::PointXYZ> &cloud_cam);
};