#pragma once

#include <cw2_world_spawner/Task1Service.h>
#include <geometry_msgs/PointStamped.h>
#include <string>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

namespace task1 {

  bool solve(const cw2_world_spawner::Task1Service::Request &req,
             cw2_world_spawner::Task1Service::Response &res);
}