#include "cw2_class.h"
#include "task1.h"

namespace task1 {

  pcl::PointCloud<pcl::PointXYZ>::Ptr capturePointCloud() {
    ROS_INFO("Waiting for point cloud...");

    // Use waitForMessage with a timeout (e.g., 5 seconds)
    boost::shared_ptr<const sensor_msgs::PointCloud2> msg =
        ros::topic::waitForMessage<sensor_msgs::PointCloud2>("/camera/depth_registered/points", ros::Duration(5.0));

    if (!msg) {
        ROS_WARN("No point cloud received from /camera/depth_registered/points, trying alternative topic...");

        // Attempt to get the point cloud from an alternative topic
        msg = ros::topic::waitForMessage<sensor_msgs::PointCloud2>("/r200/camera/depth_registered/points", ros::Duration(5.0));

        if (!msg) {
          ROS_ERROR("No point cloud received from both topics within timeout.");
          return nullptr;
        }
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*msg, *cloud);

    ROS_INFO("Point cloud received successfully.");
    return cloud;
}

  geometry_msgs::Quaternion computeOrientation(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
                          const geometry_msgs::Pose object_pose,
                          const std::string shape_type)
  {
    // Downsampling using Voxel Grid Filter
    pcl::VoxelGrid<pcl::PointXYZ> voxel;
    voxel.setInputCloud(cloud);
    voxel.setLeafSize(0.005f, 0.005f, 0.005f);
    voxel.filter(*cloud);

    // Compute the oriented bounding box using MomentOfInertiaEstimation.
    pcl::MomentOfInertiaEstimation<pcl::PointXYZ> feature_extractor;
    feature_extractor.setInputCloud(cloud);
    feature_extractor.compute();

    pcl::PointXYZ min_point_OBB, max_point_OBB, position_OBB;
    Eigen::Matrix3f rotational_matrix_OBB;
    feature_extractor.getOBB(min_point_OBB, max_point_OBB, position_OBB, rotational_matrix_OBB);

    // Extract the angle of rotation about the z-axis.
    // For a flat object, we assume that rotation about z is dominant.
    float angle = std::atan2(rotational_matrix_OBB(1,0), rotational_matrix_OBB(0,0));

    // Perform Principal Component Analysis (PCA)
    // pcl::PCA<pcl::PointXYZ> pca;
    // pca.setInputCloud(cloud);

    // Compute the orientation of the object based on its point cloud and shape type.
    // tf2::Quaternion quat;
    // if (shape_type == "nought") {
    //   quat.setRPY(M_PI, 0, -M_PI / 4);
    // }

    // else if (shape_type == "cross"){
    //   quat.setRPY(M_PI, 0, -M_PI / 4);
    // }
    // else {quat.setRPY(M_PI, 0, -M_PI / 4);}
    tf2::Quaternion quat;
    quat.setRPY(M_PI, 0, angle);
    geometry_msgs::Quaternion orientation = tf2::toMsg(quat);
    return orientation;
  } 


  bool solve(const cw2_world_spawner::Task1Service::Request &req,
             cw2_world_spawner::Task1Service::Response &res)
  {
    ROS_INFO("[Task1] Solving Task 1...");

    // Extract the object and goal points, and the shape type from the service request.
    geometry_msgs::PointStamped object_point = req.object_point;
    geometry_msgs::PointStamped goal_point   = req.goal_point;
    std::string shape_type = req.shape_type;

    // Create an instance of the cw2 class to access moveArm, moveGripper, etc.
    ros::NodeHandle nh;
    cw2 robot(nh);  // Pass the NodeHandle to the constructor

    // Move the robot arm to the object point.
    geometry_msgs::Pose observe_pose;
    observe_pose.position = object_point.point;
    observe_pose.position.z += 0.5;  // Adjust height for observing

    tf2::Quaternion quat2;
    quat2.setRPY(M_PI, 0, -M_PI / 4);
    geometry_msgs::Quaternion original_orientation = tf2::toMsg(quat2);
    observe_pose.orientation = original_orientation;

    if (!robot.moveArm(observe_pose)) {
      ROS_ERROR("Failed to move to observe pose");
      return false;
    }

    // Get the current point cloud.
    // (Assume your cw2 class stores the latest cloud and provides a getter.)
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = capturePointCloud();
    

    // Compute the orientation of the object based on its point cloud and shape.
    geometry_msgs::Quaternion computed_orientation = computeOrientation(cloud, observe_pose, shape_type);
    
    // Extract yaw angle from computed_orientation.
    tf2::Quaternion q;
    tf2::fromMsg(computed_orientation, q);
    double roll, pitch, yaw;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

    // Construct the object pose (for picking).
    geometry_msgs::Pose object_pose;
    object_pose.position = object_point.point;
    // Use the computed orientation instead of a fixed one.
    object_pose.orientation = computed_orientation;

    // Adjust the height for grasping.
    object_pose.position.z += 0.04;

    // Define the pick offset (80 mm) and adjust the pick pose along the computed direction.
    double pick_offset = 0.08; // 80mm = 0.08 m
    // Use the computed angle to determine offset components.
    object_pose.position.x += pick_offset * cos(yaw);
    object_pose.position.y += pick_offset * sin(yaw);

    // Attempt to pick up the object.
    if (!robot.pick(object_pose)) {
      ROS_ERROR("Failed to pick the object");
      return false;
    }

    // Construct the goal pose (for placing).
    geometry_msgs::Pose goal_pose;
    goal_pose.position = goal_point.point;
    goal_pose.position.z += 0.2;  // Adjust height for placing
    goal_pose.orientation = original_orientation;  // Use the same orientation as the object

    // Attempt to place the object in the basket.
    if (!robot.place(goal_pose)) {
      ROS_ERROR("Failed to place the object in the basket");
      return false;
    }

    ROS_INFO("Task 1 completed successfully");
    return true;
  }

} // namespace task1
