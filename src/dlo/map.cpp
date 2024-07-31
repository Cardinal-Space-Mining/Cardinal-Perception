/************************************************************
 *
 * Copyright (c) 2022, University of California, Los Angeles
 *
 * Authors: Kenny J. Chen, Brett T. Lopez
 * Contact: kennyjchen@ucla.edu, btlopez@ucla.edu
 *
 ***********************************************************/

#include "dlo/map.h"
#include "util.hpp"

#include <functional>
#include <memory>

#include <pcl_conversions/pcl_conversions.h>


/** Constructor */
dlo::MapNode::MapNode() : Node("dlo_map_node")
{
    this->getParams();

    this->keyframe_sub = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "keyframes", 10, std::bind(&dlo::MapNode::keyframeCB, this, std::placeholders::_1));
    this->map_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>("map", 100);

    this->dlo_map = std::make_shared<pcl::PointCloud<dlo::PointType>>();
}

/** Destructor */
dlo::MapNode::~MapNode() {}

/** Start Map Node */
void dlo::MapNode::start()
{
    RCLCPP_INFO(this->get_logger(), "Starting DLO Map Node");
}

/** Get Params */
void dlo::MapNode::getParams()
{
    dlo::declare_param<std::string>(this, "dlo/odomNode/odom_frame", this->odom_frame, "odom");
    dlo::declare_param<bool>(this, "dlo/mapNode/publishFullMap", this->publish_full_map_, true);
    dlo::declare_param<double>(this, "dlo/mapNode/publishFreq", this->publish_freq_, 1.0);
    dlo::declare_param<double>(this, "dlo/mapNode/leafSize", this->leaf_size_, 0.5);
}

/** Node Callback */
void dlo::MapNode::keyframeCB(const sensor_msgs::msg::PointCloud2::ConstSharedPtr & keyframe)
{
    // convert scan to pcl format
    pcl::PointCloud<dlo::PointType>::Ptr keyframe_pcl = std::make_shared<pcl::PointCloud<dlo::PointType>>();
    pcl::fromROSMsg(*keyframe, *keyframe_pcl);

    // voxel filter
    this->voxelgrid.setLeafSize(this->leaf_size_, this->leaf_size_, this->leaf_size_);
    this->voxelgrid.setInputCloud(keyframe_pcl);
    this->voxelgrid.filter(*keyframe_pcl);

    // save keyframe to map
    this->map_stamp = keyframe->header.stamp;
    *this->dlo_map += *keyframe_pcl;

    if(keyframe_pcl->points.size() == keyframe_pcl->width * keyframe_pcl->height)
    {
        sensor_msgs::msg::PointCloud2 map_ros;
        pcl::toROSMsg(*keyframe_pcl, map_ros);
        map_ros.header.stamp = this->now();
        map_ros.header.frame_id = this->odom_frame;
        this->map_pub->publish(map_ros);
    }
}
