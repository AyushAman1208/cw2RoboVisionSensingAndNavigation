#include "task3.h"
#include <ros/ros.h>

namespace task3 {

bool solve(const cw2_world_spawner::Task3Service::Request &req,
           cw2_world_spawner::Task3Service::Response &res)
{
  ROS_INFO("[Task3] Solving Task 3...");
  return true;
}

}