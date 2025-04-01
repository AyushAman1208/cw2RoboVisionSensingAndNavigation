/* feel free to change any part of this file, or delete this file. In general,
you can do whatever you want with this template code, including deleting it all
and starting from scratch. The only requirment is to make sure your entire 
solution is contained within the cw2_team_<your_team_number> package */

#include "cw2_class.h" // change to your team name here!
#include "task1.h"
#include "task2.h"
#include "task3.h"

///////////////////////////////////////////////////////////////////////////////

cw2::cw2(ros::NodeHandle nh) : nh_(nh) {
  t1_service_ = nh_.advertiseService("/task1_start", &cw2::t1_callback, this);
  t2_service_ = nh_.advertiseService("/task2_start", &cw2::t2_callback, this);
  t3_service_ = nh_.advertiseService("/task3_start", &cw2::t3_callback, this);
  set_arm_service_ = nh_.advertiseService("/set_arm", &cw2::setArmCallback, this);
  set_gripper_service_ = nh_.advertiseService("/set_gripper", &cw2::setGripperCallback, this);
  pick_service_ = nh_.advertiseService("/pick", &cw2::pickCallback, this);
  place_service_ = nh_.advertiseService("/place", &cw2::placeCallback, this);
  ROS_INFO("cw2 class initialised");
}

///////////////////////////////////////////////////////////////////////////////

bool
cw2::t1_callback(cw2_world_spawner::Task1Service::Request &request,
  cw2_world_spawner::Task1Service::Response &response) 
{
  /* function which should solve task 1 */

  ROS_INFO("The coursework solving callback for task 1 has been triggered");
  return task1::solve(request, response);

  return true;
}

///////////////////////////////////////////////////////////////////////////////

bool
cw2::t2_callback(cw2_world_spawner::Task2Service::Request &request,
  cw2_world_spawner::Task2Service::Response &response)
{
  /* function which should solve task 2 */

  ROS_INFO("The coursework solving callback for task 2 has been triggered");
  return task2::solve(request, response);

  return true;
}

///////////////////////////////////////////////////////////////////////////////

bool
cw2::t3_callback(cw2_world_spawner::Task3Service::Request &request,
  cw2_world_spawner::Task3Service::Response &response)
{
  /* function which should solve task 3 */

  ROS_INFO("The coursework solving callback for task 3 has been triggered");
  return task3::solve(request, response);

  return true;
}

///////////////////////////////////////////////////////////////////////////////
bool
cw2::setArmCallback(cw2_team_4::set_arm::Request &request,
 cw2_team_4::set_arm::Response &response)
{
 // set arm position, true if sucessful
 response.success = moveArm(request.pose);


 return response.success;
}


////////////////////////////////////////////////////////////////////////////////
bool
cw2::setGripperCallback(cw2_team_4::set_gripper::Request &request,
 cw2_team_4::set_gripper::Response &response)
{
 // set gripper fingers to a specific width
 response.success = moveGripper(request.finger_distance);


 return response.success; 
}


////////////////////////////////////////////////////////////////////////////////
bool
cw2::pickCallback(cw2_team_4::Pick::Request &request,
 cw2_team_4::Pick::Response &response)
{
 bool success = pick(request.object_loc);


 return success;
 }


////////////////////////////////////////////////////////////////////////////////
bool
cw2::placeCallback(cw2_team_4::Place::Request &request,
 cw2_team_4::Place::Response &response)
{
 bool success = place(request.goal_loc);


 return success;
 }

////////////////////////////////////////////////////////////////////////////////
bool
cw2::moveArm(const geometry_msgs::Pose &target_pose)
{
 // setup the target pose
 ROS_INFO("Setting pose target");
 arm_group_.setPoseTarget(target_pose);


 // create a movement plan for the arm
 ROS_INFO("Attempting to plan the path");
 moveit::planning_interface::MoveGroupInterface::Plan my_plan;
 bool success = (arm_group_.plan(my_plan) ==
   moveit::planning_interface::MoveItErrorCode::SUCCESS);


 // google 'c++ conditional operator' to understand this line
 ROS_INFO("Visualising plan %s", success ? "" : "FAILED");


 // execute the planned path
 arm_group_.move();


 return success;
}

////////////////////////////////////////////////////////////////////////////////
bool
cw2::moveGripper(float width)
{
 // safety checks in case width exceeds safe values
 if (width > gripper_open_)
   width = gripper_open_;
 if (width < gripper_closed_)
   width = gripper_closed_;


 // calculate the joint targets as half each of the requested distance
 double eachJoint = width / 2.0;


 // create a vector to hold the joint target for each joint
 std::vector<double> gripperJointTargets(2);
 gripperJointTargets[0] = eachJoint;
 gripperJointTargets[1] = eachJoint;


 // apply the joint target
 hand_group_.setJointValueTarget(gripperJointTargets);


 // move the robot hand
 ROS_INFO("Attempting to plan the path");
 moveit::planning_interface::MoveGroupInterface::Plan my_plan;
 bool success = (hand_group_.plan(my_plan) ==
   moveit::planning_interface::MoveItErrorCode::SUCCESS);


 ROS_INFO("Visualising plan %s", success ? "" : "FAILED");


 // move the gripper joints
 hand_group_.move();


 return success;
}

/////////////////////////////////////////////////////////////////////////////////
bool cw2::pick(const geometry_msgs::Pose &object_pose) {
  // Calculate the pre-grasp pose
  geometry_msgs::Pose pre_grasp_pose = object_pose;
  pre_grasp_pose.position.z += 0.3;  // Adjust height for pre-grasp
  if (!moveGripper(gripper_open_)) return false;
  if (!moveArm(pre_grasp_pose)) return false;

  // Calculate the grasp pose (lower it a bit to grasp)
  geometry_msgs::Pose grasp_pose = object_pose;
  grasp_pose.position.z += 0.12;  // Adjust for grasping position
  if (!moveArm(grasp_pose)) return false;
  return true;
}

///////////////////////////////////////////////////////////////////////////////////
bool cw2::place(const geometry_msgs::Pose &goal_pose) {
  // Calculate the pre-place pose
  geometry_msgs::Pose pre_place_pose = goal_pose;
  pre_place_pose.position.z += 0.3;  // Adjust height for pre-place
  if (!moveArm(pre_place_pose)) return false;

  // Calculate the place pose (lower it a bit to place the object)
  geometry_msgs::Pose place_pose = goal_pose;
  place_pose.position.z += 0.12;  // Adjust for placing position
  if (!moveArm(place_pose)) return false;
  if (!moveGripper(gripper_open_)) return false;

  // Return to the pre-place position
  return moveArm(pre_place_pose);
}

