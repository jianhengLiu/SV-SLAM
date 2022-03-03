/*
 * @Author: Jianheng Liu
 * @Date: 2021-10-24 17:32:22
 * @LastEditors: Jianheng Liu
 * @LastEditTime: 2021-12-06 15:31:12
 * @Description: Description
 */

#include "sv_slam.h"

using namespace std;

SV_SLAM::SV_SLAM(ros::NodeHandle &nh)
{
  std::string sensor_config_file = "/home/chrisliu/ROSws/SV-SLAM_ws/src/SV-SLAM/config/sv_slam_config.yaml";
  // std::string sensor_config_file;
  // nh.getParam("sensor_config_file", sensor_config_file);

  readParameters(sensor_config_file);
  sensor = std::make_shared<Sensor>(sensor_config_file);
  // map = std::make_shared<Map>(nh, sensor_config_file);

  RegisterSubscriber(nh);
  // RegisterPublisher(nh);
}

void SV_SLAM::readParameters(const std::string &sensor_config_file)
{
  cv::FileStorage fsSettings(sensor_config_file, cv::FileStorage::READ);
  if (!fsSettings.isOpened())
  {
    std::cerr << "ERROR: Wrong path to settings" << std::endl;
  }
  fsSettings["lidar_topic"] >> LIDAR_TOPIC;

  fsSettings.release();
}

void SV_SLAM::RegisterSubscriber(ros::NodeHandle &nh)
{
  lidar_sub_ = nh.subscribe(LIDAR_TOPIC, 100, &SV_SLAM::lidar_callback, this);
  // gps_pos_sub_ = nh.subscribe("/kitti/oxts/gps/fix", 100,
  // &SV_SLAM::gps_pos_callback, this); gps_vel_sub_ =
  // nh.subscribe("/kitti/oxts/gps/vel", 100, &SV_SLAM::gps_vel_callback, this);
  for (int i = 0; i < sensor->camera.size(); ++i)
    img_sub_.emplace_back(nh.subscribe<sensor_msgs::Image>(
        sensor->camera[i].IMAGE_TOPIC, 100,
        boost::bind(&SV_SLAM::img_callback, this, _1, i)));
}

void SV_SLAM::RegisterPublisher(ros::NodeHandle &nh) {}

void SV_SLAM::lidar_callback(const sensor_msgs::PointCloud2ConstPtr &pcl_msg)
{
  sensor->lidar->input(pcl_msg);
}

void SV_SLAM::gps_pos_callback(
    const sensor_msgs::NavSatFixConstPtr &gps_pos_msg)
{
  sensor->gps->input_pos(gps_pos_msg);
}

void SV_SLAM::gps_vel_callback(
    const geometry_msgs::TwistStampedConstPtr &gps_vel_msg)
{
  sensor->gps->input_vel(gps_vel_msg);
}

void SV_SLAM::img_callback(const sensor_msgs::ImageConstPtr &_img_msg,
                           const int _cam_id)
{
  sensor->camera[_cam_id].input_img(_img_msg);
}