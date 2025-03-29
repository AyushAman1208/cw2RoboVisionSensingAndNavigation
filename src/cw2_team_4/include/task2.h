#pragma once

#include <cw2_world_spawner/Task2Service.h>

namespace task2 {
  bool solve(const cw2_world_spawner::Task2Service::Request &req,
             cw2_world_spawner::Task2Service::Response &res);
}