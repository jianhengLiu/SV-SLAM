/*
 * @Author: Jianheng Liu
 * @Date: 2021-10-24 17:32:12
 * @LastEditors: Jianheng Liu
 * @LastEditTime: 2021-11-20 21:18:49
 * @Description: Description
 */
#pragma once

#include <ros/ros.h>

#include "sensor/sensor.h"
#include "map/map.h"

class SV_SLAM {
public:
  SV_SLAM(ros::NodeHandle &nh);

private:
  std::shared_ptr<Sensor> sensor;
  std::shared_ptr<Map> map;

  ros::Subscriber lidar_sub_, gps_pos_sub_, gps_vel_sub_;
  std::vector<ros::Subscriber> img_sub_;
  std::string LIDAR_TOPIC;

  void readParameters(const std::string &sensor_config_file);

  void RegisterSubscriber(ros::NodeHandle &nh);

  void RegisterPublisher(ros::NodeHandle &nh);

  void lidar_callback(const sensor_msgs::PointCloud2ConstPtr &pcl_msg);
  void gps_pos_callback(const sensor_msgs::NavSatFixConstPtr &gps_pos_msg);
  void gps_vel_callback(const geometry_msgs::TwistStampedConstPtr &gps_vel_msg);
  void img_callback(const sensor_msgs::ImageConstPtr &_img_msg,
                    const int _cam_id);
};