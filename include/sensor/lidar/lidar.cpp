/*
 * @Author: Jianheng Liu
 * @Date: 2021-10-24 16:43:48
 * @LastEditors: Jianheng Liu
 * @LastEditTime: 2021-11-17 22:27:21
 * @Description: Description
 */
#include "lidar.h"

using namespace std;
void Lidar::input(const sensor_msgs::PointCloud2ConstPtr &pcl_msg)
{
    m_buf_.lock();
    lidar_buf_.push(pcl_msg);
    m_buf_.unlock();
    cond_lidar.notify_one();
}

sensor_msgs::PointCloud2ConstPtr Lidar::getpopLidarMsg()
{
    sensor_msgs::PointCloud2ConstPtr lidar_msg;
    std::unique_lock<std::mutex> locker(m_buf_);
    cond_lidar.wait(locker, [this]()
                    { return !lidar_buf_.empty(); });
    lidar_msg = lidar_buf_.front();
    lidar_buf_.pop();
    locker.unlock();
    return lidar_msg;
}

void Lidar::removeClosePointCloud(const pcl::PointCloud<pcl::PointXYZ> &cloud_in,
                                  pcl::PointCloud<pcl::PointXYZ> &cloud_out, float thres)
{
    if (&cloud_in != &cloud_out)
    {
        cloud_out.header = cloud_in.header;
        cloud_out.points.resize(cloud_in.points.size());
    }

    size_t j = 0;

    for (auto point : cloud_in.points)
    {
        if (point.x * point.x + point.y * point.y + point.z * point.z < thres * thres)
            continue;
        cloud_out.points[j] = point;
        j++;
    }
    if (j != cloud_in.points.size())
    {
        cloud_out.points.resize(j);
    }

    cloud_out.height = 1;
    cloud_out.width = static_cast<uint32_t>(j);
    cloud_out.is_dense = true;
}