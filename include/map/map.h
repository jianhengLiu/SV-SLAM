/*
 * @Author: Jianheng Liu
 * @Date: 2021-10-24 16:46:12
 * @LastEditors: Jianheng Liu
 * @LastEditTime: 2021-12-05 20:20:00
 * @Description: Description
 */
#pragma once

#include <ros/ros.h>


#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <eigen3/Eigen/Dense>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/octree/octree_search.h>

// Line3Dpp
#include "line3D.h"

class Map
{
public:
  Map(ros::NodeHandle &nh, const std::string &sensor_config_file);

  visualization_msgs::Marker getLineMap();
  void pubLineMapfromTXT(const std::string &line3D_file_path);

private:
  ros::NodeHandle node_;
  ros::Publisher line_map_pub_;


  pcl::PointCloud<pcl::PointXYZ>::Ptr global_map_{
      new pcl::PointCloud<pcl::PointXYZ>};
  bool HAS_MAP;
  std::string MAP_PATH;
  double OCTREE_RESOLUTION;

  std::vector<L3DPP::FinalLine3D> L3DPP_result_;

  void sensorProcess();

  void readParameters(const std::string &sensor_config_file);

  void constructSemanticLineMapfromColmap();
};