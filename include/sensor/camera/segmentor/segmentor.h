/*
 * @Author: Jianheng Liu
 * @Date: 2021-10-31 23:10:22
 * @LastEditors: Jianheng Liu
 * @LastEditTime: 2021-12-06 23:45:35
 * @Description: Description
 */
#pragma once

#include <opencv2/opencv.hpp>

#include <sensor_msgs/Image.h>

class Segmentor
{
public:
    Segmentor(const std::string &sensor_config_file);

    cv::Mat infer(cv::Mat &input_img);

private:
    std::string CONFIG_PATH, CHECKPOINT_PATH, TORCH_SCRIPT_PATH, DEVICE;
    bool SHOW_IMG;

    void readParameters(const std::string &sensor_config_file);
};