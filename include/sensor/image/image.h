/*
 * @Author: Jianheng Liu
 * @Date: 2021-10-24 16:43:34
 * @LastEditors: Jianheng Liu
 * @LastEditTime: 2021-11-17 23:01:20
 * @Description: Description
 */
#pragma once

#include <stdio.h>
#include <queue>
#include <mutex>
#include <thread>
#include <condition_variable>

#include <eigen3/Eigen/Dense>

#include <sensor_msgs/Image.h>

#include "segmentor/segmentor.h"

struct CamParam
{
    std::string model_type;
    int width;
    int height;

    Eigen::Matrix4d R_rect = Eigen::Matrix4d::Identity();
    Eigen::Matrix<double, 3, 4> Proj;
};

class Image
{
public:
    Image(const std::string &sensor_config_file);

    void input_left(const sensor_msgs::ImageConstPtr &img_left_msg);
    void input_right(const sensor_msgs::ImageConstPtr &img_right_msg);
    
    sensor_msgs::ImageConstPtr getpopCam0Msg();

    std::condition_variable cond_seg;
    
    std::queue<cv::Mat> seg_buf;

    std::vector<CamParam> cam_param_;

private:
    std::shared_ptr<Segmentor> segmentor;

    std::string sensor_config_file_;
    bool ENABLE_SEGMENTOR;

    std::mutex m_left_, m_right_;
    std::thread segmentor_thread_;
    std::condition_variable cond_cam0_;

    std::queue<sensor_msgs::ImageConstPtr> cam0_buf_;
    std::queue<sensor_msgs::ImageConstPtr> cam1_buf_;

    void segmentorProcess();

    void readParameters(const std::string &sensor_config_file);
};