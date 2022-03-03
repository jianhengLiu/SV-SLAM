/*
 * @Author: Jianheng Liu
 * @Date: 2021-10-24 17:42:38
 * @LastEditors: Jianheng Liu
 * @LastEditTime: 2021-12-03 15:48:46
 * @Description: Description
 */


// #define BACKWARD_HAS_DW 1
// #include "backward.hpp"
// namespace backward {
// backward::SignalHandling sh;
// }

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