/* feel free to change any part of this file, or delete this file. In general,
you can do whatever you want with this template code, including deleting it all
and starting from scratch. The only requirment is to make sure your entire 
solution is contained within the cw2_team_<your_team_number> package */

// include guards, prevent .h file being defined multiple times (linker error)
#ifndef cw2_CLASS_H_
#define cw2_CLASS_H_

// system includes
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointCloud.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>

// PCL includes
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/common.h>
#include <pcl/common/centroid.h>
#include <pcl/common/pca.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/features/moment_of_inertia_estimation.h>

// standard c++ library includes (std::string, std::vector)
#include <string>
#include <vector>
#include <iostream>
#include <cmath>
#include <Eigen/Dense>
#include <algorithm>
#include <limits>

// include services from the spawner package - we will be responding to these
#include "cw2_world_spawner/Task1Service.h"
#include "cw2_world_spawner/Task2Service.h"
#include "cw2_world_spawner/Task3Service.h"

// include any services created in this package
#include "cw2_team_4/set_arm.h"
#include "cw2_team_4/set_gripper.h"
#include "cw2_team_4/Pick.h"
#include "cw2_team_4/Place.h"



class cw2
{
public:

  /* ----- class member functions ----- */

  // constructor
  cw2();
  cw2(ros::NodeHandle nh);

  bool t1_callback(cw2_world_spawner::Task1Service::Request &request,
                   cw2_world_spawner::Task1Service::Response &response);
  bool t2_callback(cw2_world_spawner::Task2Service::Request &request,
                   cw2_world_spawner::Task2Service::Response &response);
  bool t3_callback(cw2_world_spawner::Task3Service::Request &request,
                   cw2_world_spawner::Task3Service::Response &response);

  bool setArmCallback(cw2_team_4::set_arm::Request &request,
                      cw2_team_4::set_arm::Response &response);
  bool setGripperCallback(cw2_team_4::set_gripper::Request &request,
                           cw2_team_4::set_gripper::Response &response);
  bool pickCallback(cw2_team_4::Pick::Request &request,
                    cw2_team_4::Pick::Response &response);
  bool placeCallback(cw2_team_4::Place::Request &request,
                     cw2_team_4::Place::Response &response);

  bool moveArm(const geometry_msgs::Pose &target_pose);
  bool moveGripper(float width);
  bool pick(const geometry_msgs::Pose &object_loc);
  bool place(const geometry_msgs::Pose &goal_loc);


  // Define constant values
  std::string base_frame_ = "panda_link0";
  float gripper_open_ = 80e-3;
  float gripper_closed_ = 0.0;  

  /* ----- class member variables ----- */

  ros::NodeHandle nh_;
  ros::ServiceServer t1_service_;
  ros::ServiceServer t2_service_;
  ros::ServiceServer t3_service_;
  ros::ServiceServer set_arm_service_;
  ros::ServiceServer set_gripper_service_;
  ros::ServiceServer pick_service_;
  ros::ServiceServer place_service_;

  moveit::planning_interface::MoveGroupInterface arm_group_{"panda_arm"};
  moveit::planning_interface::MoveGroupInterface hand_group_{"hand"};
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface_;
  
};


#endif // end of include guard for cw2_CLASS_H_