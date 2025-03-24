#include "task2.h"
#include <ros/ros.h>

namespace task2 {

bool solve(const cw2_world_spawner::Task2Service::Request &req,
           cw2_world_spawner::Task2Service::Response &res)
{
  ROS_INFO("[Task2] Solving Task 2...");
  return true;
}

}
