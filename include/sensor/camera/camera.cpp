/*
 * @Author: Jianheng Liu
 * @Date: 2021-10-24 16:43:39
 * @LastEditors: Jianheng Liu
 * @LastEditTime: 2022-03-03 10:14:00
 * @Description: Description
 */

#include "camera.h"

#include <cv_bridge/cv_bridge.h>

#include <opencv2/core/eigen.hpp>

using namespace std;

/*
 * cause: std::thread can not be copied
 * one solution:
 *    camera.emplace_back(std::move(Camera(i, sensor_config_file)));
 * */
std::thread segmentor_thread;
std::mutex m_buf;
std::condition_variable cond_cam;
std::condition_variable cond_seg;

Camera::Camera(const int _cam_id, const std::string &sensor_config_file)
    : CAM_ID(_cam_id), sensor_config_file_(sensor_config_file) {

  readParameters(sensor_config_file_);

  if (ENABLE_SEGMENTOR)
    segmentor_thread = std::thread(&Camera::segmentorProcess, this);
};

void Camera::readParameters(const std::string &sensor_config_file) {
  cv::FileStorage fsSettings(sensor_config_file, cv::FileStorage::READ);
  if (!fsSettings.isOpened()) {
    std::cerr << "ERROR: Wrong path to settings" << std::endl;
  }

  cv::FileNode fn = fsSettings["cam_config_" + to_string(CAM_ID)];

  fn["image_topic"] >> IMAGE_TOPIC;
  ENABLE_SEGMENTOR = static_cast<int>(fn["enable_segmentor"]);

  cam_param_.width = static_cast<int>(fn["image_width"]);
  cam_param_.height = static_cast<int>(fn["image_height"]);

  cv::Mat cv_R_rect;
  fn["rotation_rect_matrix"] >> cv_R_rect;
  Eigen::Matrix4d eigen_R_rect;
  cv::cv2eigen(cv_R_rect, eigen_R_rect);
  cam_param_.R_rect = eigen_R_rect;

  cv::Mat cv_Proj;
  fn["projection_matrix"] >> cv_Proj;
  Eigen::Matrix<double, 3, 4> eigen_Proj;
  cv::cv2eigen(cv_Proj, eigen_Proj);
  cam_param_.Proj = eigen_Proj;

  fn["distortion_coefficients"] >> cam_param_.distortion_coeffs;
  fn["intrinsic_matrix"] >> cam_param_.intrinsic_matrix;
  cv::initUndistortRectifyMap(cam_param_.intrinsic_matrix,
                              cam_param_.distortion_coeffs, cv::Mat(),
                              cam_param_.intrinsic_matrix,
                              cv::Size(cam_param_.width, cam_param_.height),
                              CV_16SC2, cam_param_.map1, cam_param_.map2);

  fsSettings.release();
}

void Camera::input_img(const sensor_msgs::ImageConstPtr &_img_msg) {
  m_buf.lock();
  cam_buf_.push(_img_msg);
  m_buf.unlock();
  cond_cam.notify_one();
}

sensor_msgs::ImageConstPtr Camera::getpopCamMsg() {
  sensor_msgs::ImageConstPtr cam_msg;
  std::unique_lock<std::mutex> locker(m_buf);
  while (cam_buf_.empty())
    cond_cam.wait(locker);
  cam_msg = cam_buf_.front();
  cam_buf_.pop();
  locker.unlock();
  return cam_msg;
}

void Camera::segmentorProcess() {
  segmentor = std::make_shared<Segmentor>(sensor_config_file_);

  while (1) {
    sensor_msgs::ImageConstPtr cam_msg = getpopCamMsg();

    cv_bridge::CvImagePtr cv_ptr;
    if (cam_msg->encoding == "8UC3") {
      sensor_msgs::Image img;
      img.header = cam_msg->header;
      img.height = cam_msg->height;
      img.width = cam_msg->width;
      img.is_bigendian = cam_msg->is_bigendian;
      img.step = cam_msg->step;
      img.data = cam_msg->data;
      img.encoding = "bgr8";
      cv_ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::BGR8);
    } else
      cv_ptr = cv_bridge::toCvCopy(cam_msg, sensor_msgs::image_encodings::BGR8);

    cv::Mat input_img = cv_ptr->image;

    segmentor->infer(input_img);
    // seg_buf.push(segmentor->infer(input_img));
    // cond_seg.notify_one();

    std::chrono::milliseconds dura(2);
    std::this_thread::sleep_for(dura);
  }
}

cv::Mat Camera::undistort(cv::Mat &_img_in) {
  cv::Mat img_out;
  cv::remap(_img_in, img_out, cam_param_.map1, cam_param_.map2,
            cv::INTER_LINEAR, cv::BORDER_CONSTANT, cv::Scalar());
  return img_out;
}