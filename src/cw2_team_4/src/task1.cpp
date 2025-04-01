#include "cw2_class.h"
#include "task1.h"

namespace task1 {

  // Capture a colored point cloud.
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr capturePointCloud() {
    ROS_INFO("Waiting for point cloud...");

    // Use waitForMessage with a timeout (e.g., 5 seconds)
    boost::shared_ptr<const sensor_msgs::PointCloud2> msg =
        ros::topic::waitForMessage<sensor_msgs::PointCloud2>("/camera/depth_registered/points", ros::Duration(5.0));

    if (!msg) {
      ROS_WARN("No point cloud received from /camera/depth_registered/points, trying alternative topic...");
      msg = ros::topic::waitForMessage<sensor_msgs::PointCloud2>("/r200/camera/depth_registered/points", ros::Duration(5.0));
      if (!msg) {
        ROS_ERROR("No point cloud received from both topics within timeout.");
        return nullptr;
      }
    }

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::fromROSMsg(*msg, *cloud);

    ROS_INFO("Point cloud received successfully.");
    return cloud;
  }
// Helper: Compute the four 2D corners of the OBB.
std::vector<Eigen::Vector2f> computeOBBCorners(const Eigen::Vector3f &min_pt,
  const Eigen::Vector3f &max_pt,
  const Eigen::Matrix3f &rot,
  const Eigen::Vector3f &pos)
{
std::vector<Eigen::Vector2f> corners(4);
float min_x = min_pt.x(), min_y = min_pt.y();
float max_x = max_pt.x(), max_y = max_pt.y();

Eigen::Vector3f p1(min_x, min_y, 0);
Eigen::Vector3f p2(min_x, max_y, 0);
Eigen::Vector3f p3(max_x, max_y, 0);
Eigen::Vector3f p4(max_x, min_y, 0);

// Transform from local OBB frame to sensor frame.
p1 = rot * p1 + pos;
p2 = rot * p2 + pos;
p3 = rot * p3 + pos;
p4 = rot * p4 + pos;

corners[0] = Eigen::Vector2f(p1.x(), p1.y());
corners[1] = Eigen::Vector2f(p2.x(), p2.y());
corners[2] = Eigen::Vector2f(p3.x(), p3.y());
corners[3] = Eigen::Vector2f(p4.x(), p4.y());
return corners;
}

  // Helper: Compute midpoints of consecutive points (cyclically).
  std::vector<Eigen::Vector2f> computeEdgeMidpoints(const std::vector<Eigen::Vector2f>& pts) {
  std::vector<Eigen::Vector2f> mids;
  int n = pts.size();
  for (int i = 0; i < n; i++) {
  Eigen::Vector2f mid = (pts[i] + pts[(i+1) % n]) / 2.0f;
  mids.push_back(mid);
  }
  return mids;
  }

  // Helper: Compute the centroid of a set of 2D points.
  Eigen::Vector2f computeCentroid2D(const std::vector<Eigen::Vector2f>& points) {
  Eigen::Vector2f centroid(0,0);
  for (const auto &p : points)
  centroid += p;
  return centroid / points.size();
  }

  // Helper: Sort 2D points by their polar angle around their centroid.
  void sortPointsByAngle(std::vector<Eigen::Vector2f>& points) {
  Eigen::Vector2f centroid = computeCentroid2D(points);
  std::sort(points.begin(), points.end(), [centroid](const Eigen::Vector2f &a, const Eigen::Vector2f &b) {
  return std::atan2(a.y()-centroid.y(), a.x()-centroid.x()) < std::atan2(b.y()-centroid.y(), b.x()-centroid.x());
  });
  }

  // Helper: Compute the optimal rotation (using a 2D Kabsch algorithm) that aligns two sets of corresponding points.
  float computeRotationDifference(const std::vector<Eigen::Vector2f>& P,
  const std::vector<Eigen::Vector2f>& Q)
  {
  int n = P.size();
  Eigen::Vector2f centroidP(0,0), centroidQ(0,0);
  for (int i = 0; i < n; i++) {
  centroidP += P[i];
  centroidQ += Q[i];
  }
  centroidP /= n;
  centroidQ /= n;

  Eigen::Matrix2f H = Eigen::Matrix2f::Zero();
  for (int i = 0; i < n; i++) {
  Eigen::Vector2f p = P[i] - centroidP;
  Eigen::Vector2f q = Q[i] - centroidQ;
  H += p * q.transpose();
  }

  Eigen::JacobiSVD<Eigen::Matrix2f> svd(H, Eigen::ComputeFullU | Eigen::ComputeFullV);
  Eigen::Matrix2f U = svd.matrixU();
  Eigen::Matrix2f V = svd.matrixV();
  Eigen::Matrix2f R = V * U.transpose();

  float rotation = std::atan2(R(1,0), R(0,0));
  return rotation;
  }

  // Modified computeOrientation function using 8 points (corners + edge midpoints) for model alignment.
  float computeOrientation(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
  const geometry_msgs::Pose /*object_pose*/,
  const std::string &shape_type)
  {
  // 1. Downsample using Voxel Grid Filter.
  pcl::VoxelGrid<pcl::PointXYZRGB> voxel;
  voxel.setInputCloud(cloud);
  voxel.setLeafSize(0.005f, 0.005f, 0.005f);
  voxel.filter(*cloud);

  // 2. Compute the oriented bounding box (OBB).
  pcl::MomentOfInertiaEstimation<pcl::PointXYZRGB> feature_extractor;
  feature_extractor.setInputCloud(cloud);
  feature_extractor.compute();

  pcl::PointXYZRGB min_point_OBB, max_point_OBB, position_OBB;
  Eigen::Matrix3f rotational_matrix_OBB;
  feature_extractor.getOBB(min_point_OBB, max_point_OBB, position_OBB, rotational_matrix_OBB);

  // 3. Compute 4 corners of the OBB.
  std::vector<Eigen::Vector2f> obb_corners = computeOBBCorners(
  Eigen::Vector3f(min_point_OBB.x, min_point_OBB.y, min_point_OBB.z),
  Eigen::Vector3f(max_point_OBB.x, max_point_OBB.y, max_point_OBB.z),
  rotational_matrix_OBB,
  Eigen::Vector3f(position_OBB.x, position_OBB.y, position_OBB.z)
  );

  // 4. Compute midpoints of the OBB edges.
  std::vector<Eigen::Vector2f> obb_midpoints = computeEdgeMidpoints(obb_corners);

  // 5. Combine corners and midpoints to get 8 computed points.
  std::vector<Eigen::Vector2f> computed_points = obb_corners;
  computed_points.insert(computed_points.end(), obb_midpoints.begin(), obb_midpoints.end());

  // 6. Sort computed points by polar angle.
  sortPointsByAngle(computed_points);

  // 7. Define the ideal model points.
  std::vector<Eigen::Vector2f> ideal_points;
  if (shape_type == "nought") {
  // For a square, use points on the edges:
  // Top edge: (-50, 100) and (50, 100)
  // Right edge: (100, 50) and (100, -50)
  // Bottom edge: (50, -100) and (-50, -100)
  // Left edge: (-100, -50) and (-100, 50)
  ideal_points.push_back(Eigen::Vector2f(-50, 100));
  ideal_points.push_back(Eigen::Vector2f(50, 100));
  ideal_points.push_back(Eigen::Vector2f(100, 50));
  ideal_points.push_back(Eigen::Vector2f(100, -50));
  ideal_points.push_back(Eigen::Vector2f(50, -100));
  ideal_points.push_back(Eigen::Vector2f(-50, -100));
  ideal_points.push_back(Eigen::Vector2f(-100, -50));
  ideal_points.push_back(Eigen::Vector2f(-100, 50));
  }
  else if (shape_type == "cross") {
  // For a cross, use points along the arms:
  // Horizontal arm endpoints and midpoints: (-100,0), (-50,0), (50,0), (100,0)
  // Vertical arm endpoints and midpoints: (0,100), (0,50), (0,-50), (0,-100)
  ideal_points.push_back(Eigen::Vector2f(-100, 0));
  ideal_points.push_back(Eigen::Vector2f(-50, 0));
  ideal_points.push_back(Eigen::Vector2f(50, 0));
  ideal_points.push_back(Eigen::Vector2f(100, 0));
  ideal_points.push_back(Eigen::Vector2f(0, 100));
  ideal_points.push_back(Eigen::Vector2f(0, 50));
  ideal_points.push_back(Eigen::Vector2f(0, -50));
  ideal_points.push_back(Eigen::Vector2f(0, -100));
  }

  // 8. Sort ideal points by polar angle.
  sortPointsByAngle(ideal_points);

  // 9. Compute the rotation difference using the 2D Kabsch algorithm.
  float rotation_difference = computeRotationDifference(computed_points, ideal_points);

  // 10. Combine with the baseline offset (e.g., if robot's baseline is -π/4 when unrotated).
  float final_angle = -M_PI/4 + rotation_difference;

  ROS_INFO("Rotation difference: %f, Final computed angle: %f", rotation_difference, final_angle);

  return final_angle;
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
    observe_pose.position.z += 0.42;  // Adjust height for observing
    observe_pose.position.x -= 0.04;  // Adjust x position for observing

    tf2::Quaternion quat2;
    quat2.setRPY(M_PI, 0, -M_PI / 4);
    geometry_msgs::Quaternion original_orientation = tf2::toMsg(quat2);
    observe_pose.orientation = original_orientation;

    if (!robot.moveArm(observe_pose)) {
      ROS_ERROR("Failed to move to observe pose");
      return false;
    }

    // Get the current point cloud.
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud = capturePointCloud();
    

    // Compute the orientation of the object based on its point cloud and shape.
    float rotation_angle = computeOrientation(cloud, observe_pose, shape_type);
    
    // Extract yaw angle from computed_orientation.
    tf2::Quaternion q;
    q.setRPY(M_PI, 0, rotation_angle);
    geometry_msgs::Quaternion computed_orientation = tf2::toMsg(q);

    observe_pose.orientation = computed_orientation;
    if (!robot.moveArm(observe_pose)) {
      ROS_ERROR("Failed to move to observe pose with computed orientation");
      return false;
    }

    // Construct the object pose (for picking).
    geometry_msgs::Pose object_pose;
    object_pose.position = object_point.point;
    // Use the computed orientation instead of a fixed one.
    object_pose.orientation = computed_orientation;

    // Adjust the height for grasping.
    object_pose.position.z += 0.035;

    if (shape_type == "nought") {
      object_pose.position.x += 0.08 * sin(rotation_angle + M_PI / 4);
      object_pose.position.y += 0.08 * cos(rotation_angle + M_PI / 4);
    } else if (shape_type == "cross") {
      object_pose.position.x += 0.08 * cos(rotation_angle + M_PI / 4);
      object_pose.position.y += 0.08 * sin(rotation_angle + M_PI / 4);
    }
    
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
