/*
 * @Author: Jianheng Liu
 * @Date: 2021-10-24 16:46:18
 * @LastEditors: Jianheng Liu
 * @LastEditTime: 2021-11-17 23:46:51
 * @Description: Description
 */
#include "sensor.h"

#include <opencv2/core/eigen.hpp>

#include <cv_bridge/cv_bridge.h>

#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>

using namespace std;

Sensor::Sensor(const std::string &sensor_config_file)
{
  readParameters(sensor_config_file);

  gps = std::make_shared<GPS>();
  img = std::make_shared<Image>(sensor_config_file);
  lidar = std::make_shared<Lidar>();

  sensor_thread_ = std::thread(&Sensor::sensorProcess, this);
}

void Sensor::readParameters(const std::string &sensor_config_file)
{
  cv::FileStorage fsSettings(sensor_config_file, cv::FileStorage::READ);
  if (!fsSettings.isOpened())
  {
    std::cerr << "ERROR: Wrong path to settings" << std::endl;
  }

  cv::FileNode fn = fsSettings["sensor_config"];
  cv::Mat cv_Tr_lidar2cam;
  fn["Tr_lidar2cam"] >> cv_Tr_lidar2cam;
  Eigen::Matrix4d eigen_Tr_lidar2cam;
  cv::cv2eigen(cv_Tr_lidar2cam, eigen_Tr_lidar2cam);
  sensor_param_.Tr_lidar2cam = eigen_Tr_lidar2cam;

  fsSettings.release();
}

void Sensor::sensorProcess()
{
  while (1)
  {
    sensor_msgs::PointCloud2ConstPtr lidar_msg = lidar->getpopLidarMsg();
    pcl::PointCloud<pcl::PointXYZ> cloud_in;
    pcl::fromROSMsg(*lidar_msg, cloud_in);

    // std::vector<int> indices;
    // pcl::removeNaNFromPointCloud(cloud_in, cloud_in, indices);
    // lidar->removeClosePointCloud(cloud_in, cloud_in, 0.1);

    alignLidar2Img(cloud_in, cloud_in);

    std::chrono::milliseconds dura(10);
    std::this_thread::sleep_for(dura);
  }
}

void Sensor::alignLidar2Img(const pcl::PointCloud<pcl::PointXYZ> &cloud_in,
                            pcl::PointCloud<pcl::PointXYZ> &cloud_out)
{
  cv::Mat depth_img = transformLidar2CamFrame(cloud_in, cloud_out);
  cv::imshow("depth_img", depth_img);
  cv::waitKey(1);
}

cv::Mat Sensor::transformLidar2CamFrame(
    const pcl::PointCloud<pcl::PointXYZ> &cloud_lidar,
    pcl::PointCloud<pcl::PointXYZ> &cloud_cam)
{
  if (&cloud_lidar != &cloud_cam)
  {
    cloud_cam.header = cloud_lidar.header;
    cloud_cam.points.resize(cloud_lidar.points.size());
  }

  Eigen::Matrix<double, 3, 4> Pr_lidar2cam0 = img->cam_param_[0].Proj * img->cam_param_[0].R_rect * sensor_param_.Tr_lidar2cam;

  size_t j = 0;
  // cv::Mat depth_img(img->cam_param_[0].height, img->cam_param_[0].width, CV_16UC1, cv::Scalar(0));
  cv::Mat depth_img(img->cam_param_[0].height, img->cam_param_[0].width, CV_8UC1, cv::Scalar(0));

  sensor_msgs::ImageConstPtr cam0_msg = img->getpopCam0Msg();

  cv_bridge::CvImagePtr cv_ptr;
  cv_ptr = cv_bridge::toCvCopy(cam0_msg, sensor_msgs::image_encodings::BGR8);
  cv::Mat color_img = cv_ptr->image;

  for (auto point : cloud_lidar.points)
  {
    Eigen::Vector4d point_lidar(point.x, point.y, point.z, 1);
    Eigen::Vector3d point_cam = Pr_lidar2cam0 * point_lidar;

    cv::Point2i point_uv(int(point_cam.x() / point_cam.z() + 0.5), int(point_cam.y() / point_cam.z() + 0.5));
    // Eigen::Vector2i point_uv(int(point_cam.x() / point_cam.z() + 0.5), int(point_cam.y() / point_cam.z() + 0.5));
    // Eigen::Vector2d point_uv(point_cam.x() / point_cam.z(), point_cam.y() / point_cam.z());
    if (point_uv.x >= 0 && point_uv.y >= 0 && point_uv.x < img->cam_param_[0].width && point_uv.y < img->cam_param_[0].height && point_cam.z() > 0)
    {
      // depth_img.at<unsigned char>(point_uv) = point_cam.z() * 1000;
      // depth_img.at<uchar>(point_uv) = 255 * point_cam.z() / 10;
      int depth = 255 * point_cam.z() / 30;
      cv::circle(color_img, point_uv, 1, cv::Scalar(depth, 255 - depth, depth));

      cloud_cam.points[j].x = point_cam.x();
      cloud_cam.points[j].y = point_cam.y();
      cloud_cam.points[j].z = point_cam.z();
      j++;
    }
  }
  if (j != cloud_cam.points.size())
  {
    cloud_cam.points.resize(j);
  }

  cloud_cam.height = 1;
  cloud_cam.width = static_cast<uint32_t>(j);
  cloud_cam.is_dense = true;

  return color_img;
}