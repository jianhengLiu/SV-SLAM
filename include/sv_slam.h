/*
 * @Author: Jianheng Liu
 * @Date: 2021-10-24 17:32:12
 * @LastEditors: Jianheng Liu
 * @LastEditTime: 2021-11-02 19:15:53
 * @Description: Description
 */
#pragma once

#define BACKWARD_HAS_DW 1
#include "backward.hpp"
namespace backward{
    backward::SignalHandling sh;
}

#include <ros/ros.h>

#include "sensor/sensor.h"

class SV_SLAM
{
public:
    SV_SLAM(ros::NodeHandle &nh)
    {
        std::string sensor_config_file;
        nh.getParam("sensor_config_file", sensor_config_file);
        
        sensor = std::make_shared<Sensor>(sensor_config_file);

        RegisterSubscriber(nh);
        RegisterPublisher(nh);
    };

private:
    std::shared_ptr<Sensor> sensor;

    ros::Subscriber lidar_sub_, gps_pos_sub_, gps_vel_sub_, img_left_sub_, img_right_sub_;

    void RegisterSubscriber(ros::NodeHandle &nh);

    void RegisterPublisher(ros::NodeHandle &nh);

    void lidar_callback(const sensor_msgs::PointCloud2ConstPtr &pcl_msg);
    void gps_pos_callback(const sensor_msgs::NavSatFixConstPtr &gps_pos_msg);
    void gps_vel_callback(const geometry_msgs::TwistStampedConstPtr &gps_vel_msg);
    void img_left_callback(const sensor_msgs::ImageConstPtr &img_left_msg);
    void img_right_callback(const sensor_msgs::ImageConstPtr &img_right_msg);
};