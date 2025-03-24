#include "task1.h"
#include <ros/ros.h>

namespace task1 {

bool solve(const cw2_world_spawner::Task1Service::Request &req,
           cw2_world_spawner::Task1Service::Response &res)
{
  ROS_INFO("[Task1] Solving Task 1...");
  return true;
}

}
