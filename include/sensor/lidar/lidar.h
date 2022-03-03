/*
 * @Author: Jianheng Liu
 * @Date: 2021-10-24 16:43:44
 * @LastEditors: Jianheng Liu
 * @LastEditTime: 2021-11-17 22:28:30
 * @Description: Description
 */

#pragma once

#include <stdio.h>
#include <queue>
#include <mutex>
#include <condition_variable>

#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>

#include <sensor_msgs/PointCloud2.h>

class Lidar
{
public:
    Lidar(){};
    
    void input(const sensor_msgs::PointCloud2ConstPtr &pcl_msg);

    sensor_msgs::PointCloud2ConstPtr getpopLidarMsg();

   void removeClosePointCloud(const pcl::PointCloud<pcl::PointXYZ> &cloud_in,                               pcl::PointCloud<pcl::PointXYZ> &cloud_out, float thres);

    std::condition_variable cond_lidar;

private:
    std::mutex m_buf_;
    std::queue<sensor_msgs::PointCloud2ConstPtr> lidar_buf_;
};