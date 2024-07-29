/************************************************************
 *
 * Copyright (c) 2022, University of California, Los Angeles
 *
 * Authors: Kenny J. Chen, Brett T. Lopez
 * Contact: kennyjchen@ucla.edu, btlopez@ucla.edu
 *
 ***********************************************************/

#include "dlo/dlo.h"
#include "rclcpp/rclcpp.hpp"

#include <string>

#include <pcl/filters/voxel_grid.h>

#include <sensor_msgs/msg/point_cloud2.hpp>


class dlo::MapNode: public rclcpp::Node
{
public:
  MapNode();
  ~MapNode();

  void start();

private:
  void getParams();
  void keyframeCB(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& keyframe);

private:
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr keyframe_sub;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr map_pub;

  pcl::PointCloud<dlo::PointType>::Ptr dlo_map;
  pcl::VoxelGrid<dlo::PointType> voxelgrid;

  rclcpp::Time map_stamp;
  std::string odom_frame;

  bool publish_full_map_;
  double publish_freq_;
  double leaf_size_;

};
