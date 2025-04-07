#pragma once

#include <cw2_world_spawner/Task1Service.h>
#include <geometry_msgs/PointStamped.h>
#include <string>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

namespace task1 {

  bool solve(geometry_msgs::PointStamped object_point,geometry_msgs::PointStamped goal_point, std::string shape_type,
    cw2 &robot, ros::NodeHandle &nh,std::string currTask);
}