#pragma once

#include <cw2_world_spawner/Task3Service.h>

#include "cw2_class.h"
#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <geometry_msgs/Pose.h>
namespace task3{
  bool solve(const cw2_world_spawner::Task3Service::Request &req,
             cw2_world_spawner::Task3Service::Response &res, cw2& robot, ros::NodeHandle &nh);
}