/*
 * @Author: Jianheng Liu
 * @Date: 2021-10-24 17:42:38
 * @LastEditors: Jianheng Liu
 * @LastEditTime: 2021-11-02 19:13:40
 * @Description: Description
 */
#include <ros/ros.h>

#include "sv_slam.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "vs_slam_node");
    ros::NodeHandle nh("~");
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info);

    SV_SLAM sv_slam(nh);

    ros::spin();
    return 0;
}