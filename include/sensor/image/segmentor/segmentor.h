/*
 * @Author: Jianheng Liu
 * @Date: 2021-10-31 23:10:22
 * @LastEditors: Jianheng Liu
 * @LastEditTime: 2021-11-02 19:43:14
 * @Description: Description
 */
#pragma once

#include <opencv2/opencv.hpp>

#include <sensor_msgs/Image.h>

class Segmentor
{
public:
    Segmentor(const std::string &sensor_config_file);

    cv::Mat infer(const cv::Mat &input_img);

private:
    std::string CONFIG_PATH, CHECKPOINT_PATH, DEVICE;
    bool SHOW_IMG;

    void readParameters(const std::string &sensor_config_file);
};