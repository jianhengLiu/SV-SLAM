/*
 * @Author: Jianheng Liu
 * @Date: 2021-10-24 16:43:00
 * @LastEditors: Jianheng Liu
 * @LastEditTime: 2021-10-25 10:41:18
 * @Description: Description
 */
#pragma once

#include <stdio.h>
#include <queue>
#include <mutex>

#include <sensor_msgs/NavSatFix.h>
#include <geometry_msgs/TwistStamped.h>

class GPS
{
public:
    GPS(){};

    void input_pos(const sensor_msgs::NavSatFixConstPtr &gps_pos_msg);
    void input_vel(const geometry_msgs::TwistStampedConstPtr &gps_vel_msg);

private:
    std::mutex m_pos_, m_vel_;
    std::queue<sensor_msgs::NavSatFixConstPtr> pos_buf_;
    std::queue<geometry_msgs::TwistStampedConstPtr> vel_buf_;
};