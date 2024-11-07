/*
 * Copyright (c) 2019 Intel Corporation
 *
 * This work is licensed under the terms of the MIT license.
 * For a copy, see <https://opensource.org/licenses/MIT>.
 */
#include "PclRecorder.h"
#include <string>
#include <pcl/io/pcd_io.h>
#include <pcl_ros/transforms.h>
#include <sstream>

PclRecorder::PclRecorder()
{
  tfListener = new tf2_ros::TransformListener(tf_buffer_);

  if (mkdir("/home/rbl/project/carla_map/town03", 0777) == -1) {
    ROS_WARN("Could not create directory!");
  }

  // Create a ROS subscriber for the input point cloud
  std::string roleName;
  if (!ros::param::get("~role_name", roleName)) {
    roleName = "ego_vehicle";
  }
  // sub = nh.subscribe("/carla/" + roleName + "/lidar", 1, &PclRecorder::callback, this);
  sub = nh.subscribe("/carla/" + roleName + "/semantic_lidar", 1, &PclRecorder::callbackSemantic, this);
  // 订阅车辆的位姿信息
  // odom_sub_ = nh.subscribe("/carla/" + roleName + "/odometry", 1, &PclRecorder::odomCallback, this);
}

// void PclRecorder::odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
// {
//   // 提取车辆的位置
//   vehicle_position_ = Eigen::Vector3d(msg->pose.pose.position.x,
//                                       msg->pose.pose.position.y,
//                                       msg->pose.pose.position.z);

//   // 提取车辆的方向（四元数）
//   vehicle_orientation_ = Eigen::Quaterniond(msg->pose.pose.orientation.w,
//                                             msg->pose.pose.orientation.x,
//                                             msg->pose.pose.orientation.y,
//                                             msg->pose.pose.orientation.z);
// }

void PclRecorder::callbackSemantic(const boost::shared_ptr<const pcl::PCLPointCloud2>& cloud)
{
  if ((cloud->width * cloud->height) == 0) {
    return;
  }

  std::stringstream ss;
  ss << "/home/rbl/project/carla_map/town03/capture" << cloud->header.stamp << ".pcd";

  ROS_INFO ("Received %d data points. Storing in %s",
           (int)cloud->width * cloud->height,
           ss.str().c_str());

  Eigen::Affine3d transform;
  try {
    // 获取坐标变换
    transform = tf2::transformToEigen(tf_buffer_.lookupTransform(fixed_frame_, cloud->header.frame_id, pcl_conversions::fromPCL(cloud->header.stamp), ros::Duration(1)));

    // 创建 pcl::PointCloud<PointXYZLO> 对象
    pcl::PointCloud<PointXYZLO> pclCloud;
    
    // 将 PCLPointCloud2 转换为 PointXYZLO 类型的点云
    pcl::fromPCLPointCloud2(*cloud, pclCloud);

    // 变换点云到固定坐标系
    pcl::PointCloud<PointXYZLO> transformedCloud;
    pcl::transformPointCloud(pclCloud, transformedCloud, transform);

    // 将变换后的点云保存为 PCD 文件
    pcl::PCDWriter writer;
    writer.writeBinary(ss.str(), transformedCloud);
  }
  catch (tf2::TransformException &ex)
  {
    ROS_WARN("Could NOT transform: %s", ex.what());
  }
}




void PclRecorder::callback(const boost::shared_ptr<const pcl::PCLPointCloud2>& cloud)
{
  if ((cloud->width * cloud->height) == 0) {
    return;
  }

  std::stringstream ss;
  ss << "/tmp/pcl_capture/capture" << cloud->header.stamp << ".pcd";

  ROS_INFO ("Received %d data points. Storing in %s",
           (int)cloud->width * cloud->height,
           ss.str().c_str());

  Eigen::Affine3d transform;
  try {
    transform = tf2::transformToEigen (tf_buffer_.lookupTransform(fixed_frame_, cloud->header.frame_id,  pcl_conversions::fromPCL (cloud->header.stamp), ros::Duration(1)));

    pcl::PointCloud<pcl::PointXYZ> pclCloud;
    pcl::fromPCLPointCloud2(*cloud, pclCloud);

    pcl::PointCloud<pcl::PointXYZ> transformedCloud;
    pcl::transformPointCloud (pclCloud, transformedCloud, transform);

    pcl::PCDWriter writer;
    writer.writeBinary(ss.str(), transformedCloud);
  }
  catch (tf2::TransformException &ex)
  {
    ROS_WARN("Could NOT transform: %s", ex.what());
  }
}
