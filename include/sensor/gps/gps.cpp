/*
 * @Author: Jianheng Liu
 * @Date: 2021-10-24 16:43:09
 * @LastEditors: Jianheng Liu
 * @LastEditTime: 2021-10-25 00:03:26
 * @Description: Description
 */
#include "gps.h"
using namespace std;

void GPS::input_pos(const sensor_msgs::NavSatFixConstPtr &gps_pos_msg)
{
    m_pos_.lock();
    pos_buf_.push(gps_pos_msg);
    m_pos_.unlock();
}

void GPS::input_vel(const geometry_msgs::TwistStampedConstPtr &gps_vel_msg)
{
    m_vel_.lock();
    vel_buf_.push(gps_vel_msg);
    m_vel_.unlock();
}