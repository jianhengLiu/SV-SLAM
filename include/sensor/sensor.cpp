/*
 * @Author: Jianheng Liu
 * @Date: 2021-10-24 16:46:18
 * @LastEditors: Jianheng Liu
 * @LastEditTime: 2022-03-03 10:14:37
 * @Description: Description
 */
#include "sensor.h"

#include <opencv2/calib3d.hpp>
#include <opencv2/core/cvstd_wrapper.hpp>
#include <opencv2/core/eigen.hpp>

#include <cv_bridge/cv_bridge.h>

#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>

using namespace std;

Sensor::Sensor(const std::string &sensor_config_file) {

  readParameters(sensor_config_file);

  // gps = std::make_shared<GPS>();
  // lidar = std::make_shared<Lidar>();
  for (int i = 0; i < CAM_NUM; ++i) {
    camera.emplace_back(i, sensor_config_file);
  }

  if (ENABLE_SENSOR_ALIGN)
    sensor_align_thread_ = std::thread(&Sensor::sensorAlignProcess, this);
}

void Sensor::readParameters(const std::string &sensor_config_file) {
  cv::FileStorage fsSettings(sensor_config_file, cv::FileStorage::READ);
  if (!fsSettings.isOpened()) {
    std::cerr << "ERROR: Wrong path to settings" << std::endl;
  }

  CAM_NUM = static_cast<int>(fsSettings["cam_number"]);

  cv::FileNode fn = fsSettings["sensor_config"];
  cv::Mat cv_Tr_lidar2cam;
  fn["Tr_lidar2cam"] >> cv_Tr_lidar2cam;
  Eigen::Matrix4d eigen_Tr_lidar2cam;
  cv::cv2eigen(cv_Tr_lidar2cam, eigen_Tr_lidar2cam);
  sensor_param_.Tr_lidar2cam = eigen_Tr_lidar2cam;

  ENABLE_SENSOR_ALIGN = static_cast<int>(fn["enable_sensor_align"]);
  ENABLE_STEREO = static_cast<int>(fn["enable_stereo"]);

  fsSettings.release();
}

void Sensor::sensorAlignProcess() {
  while (1) {
    sensor_msgs::PointCloud2ConstPtr lidar_msg = lidar->getpopLidarMsg();
    pcl::PointCloud<pcl::PointXYZ> cloud_in;
    pcl::fromROSMsg(*lidar_msg, cloud_in);

    // std::vector<int> indices;
    // pcl::removeNaNFromPointCloud(cloud_in, cloud_in, indices);
    // lidar->removeClosePointCloud(cloud_in, cloud_in, 0.1);

    cout << "lidar_msg->header.stamp.toSec()" << endl
         << to_string(lidar_msg->header.stamp.toSec()) << endl;

    sensor_msgs::ImageConstPtr cam_msg = NULL;
    while (1) {
      cam_msg = camera[0].getpopCamMsg();
      if (cam_msg->header.stamp.toSec() >= lidar_msg->header.stamp.toSec()) {
        break;
      }
    }
    cout << "cam0_msg->header.stamp.toSec()" << endl
         << to_string(cam_msg->header.stamp.toSec()) << endl;

    cv_bridge::CvImagePtr cv_ptr;
    cv_ptr = cv_bridge::toCvCopy(cam_msg, sensor_msgs::image_encodings::BGR8);
    cv::Mat color_img = cv_ptr->image;

    color_img = camera[0].undistort(color_img);

    alignLidar2Img(cloud_in, color_img);

    std::chrono::milliseconds dura(10);
    std::this_thread::sleep_for(dura);
  }
}

/*
 * KITTI version
 * */
void Sensor::alignLidar2Img(const pcl::PointCloud<pcl::PointXYZ> &_cloud_in,
                            cv::Mat _image_in) {
  pcl::PointCloud<pcl::PointXYZ> cloud_cam;
  cloud_cam.header = _cloud_in.header;
  cloud_cam.points.resize(_cloud_in.points.size());

  Eigen::Matrix<double, 3, 4> Pr_lidar2cam0 = camera[0].cam_param_.Proj *
                                              camera[0].cam_param_.R_rect *
                                              sensor_param_.Tr_lidar2cam;

  size_t j = 0;
  // cv::Mat depth_img(img->cam_param_[0].height, img->cam_param_[0].width,
  // CV_16UC1, cv::Scalar(0));
  //  cv::Mat depth_img(camera[0].cam_param_.height, camera[0].cam_param_.width,
  //                    CV_8UC1, cv::Scalar(0));

  for (auto point : _cloud_in.points) {
    Eigen::Vector4d point_lidar(point.x, point.y, point.z, 1);
    Eigen::Vector3d point_cam = Pr_lidar2cam0 * point_lidar;

    cv::Point2i point_uv(int(point_cam.x() / point_cam.z() + 0.5),
                         int(point_cam.y() / point_cam.z() + 0.5));
    // Eigen::Vector2i point_uv(int(point_cam.x() / point_cam.z() + 0.5),
    // int(point_cam.y() / point_cam.z() + 0.5)); Eigen::Vector2d
    // point_uv(point_cam.x() / point_cam.z(), point_cam.y() / point_cam.z());
    if (point_uv.x >= 0 && point_uv.y >= 0 &&
        point_uv.x < camera[0].cam_param_.width &&
        point_uv.y < camera[0].cam_param_.height && point_cam.z() > 0) {
      // depth_img.at<unsigned char>(point_uv) = point_cam.z() * 1000;
      // depth_img.at<uchar>(point_uv) = 255 * point_cam.z() / 10;
      int depth = 255 * point_cam.z() / 30;
      cv::circle(_image_in, point_uv, 1, cv::Scalar(depth, 255 - depth, depth));

      cloud_cam.points[j].x = point_cam.x();
      cloud_cam.points[j].y = point_cam.y();
      cloud_cam.points[j].z = point_cam.z();
      j++;
    }
  }
  if (j != cloud_cam.points.size()) {
    cloud_cam.points.resize(j);
  }

  cloud_cam.height = 1;
  cloud_cam.width = static_cast<uint32_t>(j);
  cloud_cam.is_dense = true;

  cv::imshow("_image_in", _image_in);
  cv::waitKey(1);
}

void Sensor::matchingStereoSGBM(cv::Mat _input_left, cv::Mat _input_right) {
  int min_disparity = 0;
  int num_disparities = 128;
  int SADWindowSize = 11;
  cv::Ptr<cv::StereoSGBM> sgbm =
      cv::StereoSGBM::create(min_disparity, num_disparities, SADWindowSize);
  int P1 = 4 * imgL.channels() * SADWindowSize * SADWindowSize;
  int P2 = 32 * imgR.channels() * SADWindowSize * SADWindowSize;
  sgbm->setP1(P1);
  sgbm->setP2(P2);
  sgbm->setPreFilterCap(63);
  sgbm->setUniquenessRatio(10);
  sgbm->setSpeckleRange(32);
  sgbm->setSpeckleWindowSize(100);
  sgbm->setDisp12MaxDiff(1);
  // sgbm->setMode(cv::StereoSGBM::MODE_HH);
  cv::Mat disp32F;
  sgbm->compute(_input_left, _input_right, disp32F);
  disp32F.convertTo(disp32F, CV_32F, 1.0 / 16);
  // float* inData = disp32F.ptr<float>(284);
  // cout << float(inData[322]) << endl;
  // insertDepth32f(disp32F);
  cv::Mat disp8U = cv::Mat(disp32F.rows, disp32F.cols, CV_8UC1);
  // normalize(disp8U, disp32F, 0, 255, NORM_MINMAX, CV_8UC1);
  disp32F.convertTo(disp8U, CV_8UC1);
  cv::imshow("disparity", disp8U);
  cv::waitKey(1);

  float fx = 718.856;
  float baseline = 35;
  cv::Mat depthMap = cv::Mat::zeros(disp32F.size(), CV_32FC1);
  int height = disp32F.rows;
  int width = disp32F.cols;
  for (int k = 0; k < height; k++) {
    const float *inData = disp32F.ptr<float>(k);
    float *outData = depthMap.ptr<float>(k);
    for (int i = 0; i < width; i++) {
      if (!inData[i])
        continue;
      outData[i] = float(fx * baseline / inData[i]);
    }
  }
  // FileStorage fswrite("test.xml", FileStorage::WRITE);//
  // 新建文件，覆盖掉已有文件 fswrite << "src1" << depthMap; fswrite.release();
  cv::Mat depthMap8U = cv::Mat(depthMap.rows, depthMap.cols, CV_8UC1);
  normalize(depthMap, depthMap8U, 0, 255, cv::NORM_MINMAX, CV_8U);
  cv::imshow("depth:8U", depthMap8U);
  cv::waitKey(1);
}