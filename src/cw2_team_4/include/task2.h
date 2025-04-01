#pragma once

#include <cw2_world_spawner/Task2Service.h>
#include <geometry_msgs/PointStamped.h>
#include <string>
#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include "task2.h"
#include "cw2_class.h"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/features/moment_of_inertia_estimation.h>

namespace task2 {
  bool solve(const cw2_world_spawner::Task2Service::Request &req,
             cw2_world_spawner::Task2Service::Response &res);

  // bool takeObservation(geometry_msgs::PointStamped &observation_pose); 
}