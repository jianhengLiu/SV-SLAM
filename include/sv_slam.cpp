/*
 * @Author: Jianheng Liu
 * @Date: 2021-10-24 17:32:22
 * @LastEditors: Jianheng Liu
 * @LastEditTime: 2021-11-17 20:41:19
 * @Description: Description
 */

#include "sv_slam.h"

using namespace std;

void SV_SLAM::RegisterSubscriber(ros::NodeHandle &nh)
{
    lidar_sub_ = nh.subscribe("/kitti/velo/pointcloud", 100, &SV_SLAM::lidar_callback, this);
    // gps_pos_sub_ = nh.subscribe("/kitti/oxts/gps/fix", 100, &SV_SLAM::gps_pos_callback, this);
    // gps_vel_sub_ = nh.subscribe("/kitti/oxts/gps/vel", 100, &SV_SLAM::gps_vel_callback, this);
    img_left_sub_ = nh.subscribe("/kitti/camera_color_left/image_raw", 100, &SV_SLAM::img_left_callback, this);
    // img_right_sub_ = nh.subscribe("/kitti/camera_color_right/image_raw", 100, &SV_SLAM::img_right_callback, this);
}

void SV_SLAM::RegisterPublisher(ros::NodeHandle &nh)
{
}

void SV_SLAM::lidar_callback(const sensor_msgs::PointCloud2ConstPtr &pcl_msg)
{
    sensor->lidar->input(pcl_msg);
}

void SV_SLAM::gps_pos_callback(const sensor_msgs::NavSatFixConstPtr &gps_pos_msg)
{
    sensor->gps->input_pos(gps_pos_msg);
}

void SV_SLAM::gps_vel_callback(const geometry_msgs::TwistStampedConstPtr &gps_vel_msg)
{
    sensor->gps->input_vel(gps_vel_msg);
}

void SV_SLAM::img_left_callback(const sensor_msgs::ImageConstPtr &img_left_msg)
{
    sensor->img->input_left(img_left_msg);
}

void SV_SLAM::img_right_callback(const sensor_msgs::ImageConstPtr &img_right_msg)
{
    sensor->img->input_right(img_right_msg);
}