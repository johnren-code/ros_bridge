/*
 * Copyright (c) 2019 Intel Corporation
 *
 * This work is licensed under the terms of the MIT license.
 * For a copy, see <https://opensource.org/licenses/MIT>.
 */
#pragma once

#include <ros/ros.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_eigen/tf2_eigen.h>
// #include <nav_msgs/Odometry.h>

struct PointXYZLO {
  PCL_ADD_POINT4D;  // 添加 x, y, z 和 padding
  float cos_angle;  // 存储 CosAngle 字段
  uint32_t obj_idx; // 存储 ObjIdx 字段
  uint32_t obj_tag; // 存储 ObjTag 字段 (相当于 label)
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;  // 内存对齐
} EIGEN_ALIGN16;    // 保持16字节对齐

// 注册新的点类型到PCL
POINT_CLOUD_REGISTER_POINT_STRUCT(PointXYZLO,
                                  (float, x, x) (float, y, y) (float, z, z)
                                  (float, cos_angle, CosAngle)
                                  (uint32_t, obj_idx, ObjIdx)
                                  (uint32_t, obj_tag, ObjTag))
                                  
                                  
class PclRecorder
{
public:

  PclRecorder();

  void callback(const boost::shared_ptr<const pcl::PCLPointCloud2>& cloud);
  void callbackSemantic(const boost::shared_ptr<const pcl::PCLPointCloud2>& cloud);
  // void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);

private:
  ros::NodeHandle nh;
  ros::Subscriber sub;
  // ros::Subscriber odom_sub_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener *tfListener;
  static constexpr const char* fixed_frame_ = "map";

};
