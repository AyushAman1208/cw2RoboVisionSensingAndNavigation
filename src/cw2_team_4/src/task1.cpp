#include "task1.h"
#include "cw2_class.h"
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Point.h>
#include <ros/ros.h>
#include <tf/transform_listener.h>

namespace task1 {

  bool solve(const cw2_world_spawner::Task1Service::Request &req,
    cw2_world_spawner::Task1Service::Response &res)
{
  ROS_INFO("[Task1] Solving Task 1...");

  // Extract the object point and goal point
  geometry_msgs::PointStamped object_point = req.object_point;
  geometry_msgs::PointStamped goal_point = req.goal_point;
  std::string shape_type = req.shape_type;

  // Set robot orientation for 0,0,0,1
  tf2::Quaternion quat;
  quat.setRPY(M_PI, 0, -M_PI / 4);
  geometry_msgs::Quaternion orientation = tf2::toMsg(quat);

  // Create an instance of the cw2 class (for access to moveArm, moveGripper, etc.)
  ros::NodeHandle nh;
  cw2 robot(nh);  // Pass the NodeHandle to the constructor



  // Construct the object pose with the object point (centroid) and orientation
  geometry_msgs::Pose object_pose;
  object_pose.position = object_point.point;
  object_pose.orientation = orientation;

  object_pose.position.z += 0.035;  // Adjust height for grasping

  // We will assume that the pick location is 80mm away on the x-axis from the centroid
  double pick_offset = 0.08;  // 80mm = 0.08m

  // Calculate the pick position by adjusting the x or y-coordinate
  if (shape_type == "nought") {
    object_pose.position.y += pick_offset;
  } else if (shape_type == "cross") {
    object_pose.position.x += pick_offset;
  }

  // Pick up the object
  if (!robot.pick(object_pose)) {
  ROS_ERROR("Failed to pick the object");
  return false;
  }

  // Construct the goal pose (goal point is where we place the object)
  geometry_msgs::Pose goal_pose;
  goal_pose.position = goal_point.point;
  goal_pose.position.z += 0.2;  // Adjust height for placing

  if (shape_type == "nought") {
    goal_pose.position.y += pick_offset;
  } else if (shape_type == "cross") {
    goal_pose.position.x += pick_offset;
  }


  goal_pose.orientation = orientation;

  // Place the object in the basket
  if (!robot.place(goal_pose)) {
  ROS_ERROR("Failed to place the object in the basket");
  return false;
  }

  // Successfully completed the task
  ROS_INFO("Task 1 completed successfully");

  return true; // Return true when the task is completed
}

}