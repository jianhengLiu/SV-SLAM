/*
 * @Author: Jianheng Liu
 * @Date: 2021-10-24 16:43:39
 * @LastEditors: Jianheng Liu
 * @LastEditTime: 2021-11-17 23:01:48
 * @Description: Description
 */
#include "image.h"

#include <cv_bridge/cv_bridge.h>

#include <opencv2/core/eigen.hpp>

using namespace std;

Image::Image(const std::string &sensor_config_file)
{
  sensor_config_file_ = sensor_config_file;
  readParameters(sensor_config_file_);

  if (ENABLE_SEGMENTOR)
    segmentor_thread_ = std::thread(&Image::segmentorProcess, this);
};

void Image::readParameters(const std::string &sensor_config_file)
{
  cv::FileStorage fsSettings(sensor_config_file, cv::FileStorage::READ);
  if (!fsSettings.isOpened())
  {
    std::cerr << "ERROR: Wrong path to settings" << std::endl;
  }

  cv::FileNode fn = fsSettings["image_config"];

  ENABLE_SEGMENTOR = static_cast<int>(fn["enable_segmentor"]);

  int cam_number = static_cast<int>(fn["cam_number"]);
  for (int i = 0; i < cam_number; ++i)
  {
    CamParam cam_param;
    cv::FileNode fn_cam = fn["cam" + to_string(i)];
    fn_cam["model_type"] >> cam_param.model_type;
    cam_param.width = static_cast<int>(fn_cam["image_width"]);
    cam_param.height = static_cast<int>(fn_cam["image_height"]);

    cv::Mat cv_R_rect;
    fn_cam["rotation_rect_matrix"] >> cv_R_rect;
    Eigen::Matrix4d eigen_R_rect;
    cv::cv2eigen(cv_R_rect, eigen_R_rect);
    cam_param.R_rect = eigen_R_rect;

    cv::Mat cv_Proj;
    fn_cam["projection_matrix"] >> cv_Proj;
    Eigen::Matrix<double, 3, 4> eigen_Proj;
    cv::cv2eigen(cv_Proj, eigen_Proj);
    cam_param.Proj = eigen_Proj;
    cam_param_.emplace_back(cam_param);
  }

  fsSettings.release();
}

void Image::input_left(const sensor_msgs::ImageConstPtr &img_left_msg)
{
  m_left_.lock();
  cam0_buf_.push(img_left_msg);
  m_left_.unlock();
  cond_cam0_.notify_one();
}
void Image::input_right(const sensor_msgs::ImageConstPtr &img_right_msg)
{
  m_right_.lock();
  cam1_buf_.push(img_right_msg);
  m_right_.unlock();
}

sensor_msgs::ImageConstPtr Image::getpopCam0Msg()
{
  sensor_msgs::ImageConstPtr cam0_msg;
  std::unique_lock<std::mutex> locker(m_left_);
  cond_cam0_.wait(locker, [this]()
                  { return !cam0_buf_.empty(); });
  cam0_msg = cam0_buf_.front();
  cam0_buf_.pop();
  locker.unlock();
  return cam0_msg;
}

void Image::segmentorProcess()
{
  segmentor = std::make_shared<Segmentor>(sensor_config_file_);

  while (1)
  {
    sensor_msgs::ImageConstPtr cam0_msg = getpopCam0Msg();

    cv_bridge::CvImagePtr cv_ptr;
    cv_ptr = cv_bridge::toCvCopy(cam0_msg, sensor_msgs::image_encodings::BGR8);
    cv::Mat input_img = cv_ptr->image;

    seg_buf.push(segmentor->infer(input_img));
    cond_seg.notify_one();

    std::chrono::milliseconds dura(10);
    std::this_thread::sleep_for(dura);
  }
}