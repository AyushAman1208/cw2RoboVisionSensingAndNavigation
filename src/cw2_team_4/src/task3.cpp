#include "cw2_class.h"
#include "task3.h"

namespace task3 {

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr capturePointCloud(ros::NodeHandle &nh) { 
    ROS_INFO("Waiting for a fresh point cloud...");

    std::string topic = "/r200/camera/depth_registered/points";
    const double freshnessThreshold = 3.0; // Accept only messages published in the past 10 seconds
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    bool received = false;

    int attempts = 0;
    const int maxAttempts = 10;
    while (!received && attempts < maxAttempts) {
        boost::shared_ptr<const sensor_msgs::PointCloud2> msg =
            ros::topic::waitForMessage<sensor_msgs::PointCloud2>(topic, nh, ros::Duration(5.0));
        
        if (msg) {
            double age = (ros::Time::now() - msg->header.stamp).toSec();
            if (age <= freshnessThreshold) {
                pcl::fromROSMsg(*msg, *cloud);
                ROS_INFO("Fresh point cloud received from %s with %lu points (age: %.2f sec)",
                          topic.c_str(), cloud->points.size(), age);
                received = true;
                break;
            } else {
                ROS_WARN("Point cloud from %s is too old (%.2f sec); retrying...", topic.c_str(), age);
            }
        } else {
            ROS_WARN("No point cloud received from %s; retrying...", topic.c_str());
        }
        attempts++;
        ros::Duration(1.0).sleep();
    }

    if (!received) {
        ROS_ERROR("Failed to receive a fresh point cloud after %d attempts.", attempts);
        return nullptr;
    }

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
    const int threshold = 50;
    for (const auto &pt : cloud->points) {
        if (pt.g < threshold && (pt.r > threshold || pt.b > threshold)) {  
            filtered_cloud->push_back(pt);
        }
    }


  if (filtered_cloud->empty()) {
      ROS_WARN("Color filtering removed all points; using original cloud.");
      return cloud;
  } else {
      ROS_INFO("After color filtering, %lu points remain.", filtered_cloud->points.size());
      return filtered_cloud;
  }
  }

    // Extract clusters from a point cloud and filter clusters based on size.
// Only clusters with maximum dimension (x or y) between 0.08 m and 0.22 m are returned.
std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> extractClusters(
  const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud)
{
  // Create a KdTree for clustering.
  pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>);
  tree->setInputCloud(cloud);

  // Perform Euclidean Cluster Extraction.
  std::vector<pcl::PointIndices> clusterIndices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec;
  ec.setClusterTolerance(0.02f); // 2 cm tolerance (adjust based on your data density)
  ec.setMinClusterSize(50);      // minimum number of points in a cluster
  ec.setMaxClusterSize(25000);   // maximum number of points in a cluster
  ec.setSearchMethod(tree);
  ec.setInputCloud(cloud);
  ec.extract(clusterIndices);

  std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> clusters;
  for (const auto &indices : clusterIndices)
  {
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr cluster(new pcl::PointCloud<pcl::PointXYZRGB>);
      for (const auto &idx : indices.indices)
      {
          cluster->points.push_back(cloud->points[idx]);
      }
      cluster->width = cluster->points.size();
      cluster->height = 1;
      cluster->is_dense = true;

      // Compute the bounding box (x-y only) for the cluster.
      float minX = std::numeric_limits<float>::max();
      float maxX = -std::numeric_limits<float>::max();
      float minY = std::numeric_limits<float>::max();
      float maxY = -std::numeric_limits<float>::max();
      for (const auto &pt : cluster->points)
      {
          if (pt.x < minX) minX = pt.x;
          if (pt.x > maxX) maxX = pt.x;
          if (pt.y < minY) minY = pt.y;
          if (pt.y > maxY) maxY = pt.y;
      }
      float clusterWidth  = maxX - minX;
      float clusterHeight = maxY - minY;
      float clusterSize   = std::max(clusterWidth, clusterHeight);

      // Filter clusters to those within the tolerance: 80 mm to 220 mm.
      if (clusterSize >= 0.08f && clusterSize <= 0.22f)
      {
          clusters.push_back(cluster);
      }
  }
  return clusters;
}


// Function to transform point from camera frame to base frame
std::vector<float> transformPointCameraToBase(
  const std::vector<float>& point_camera_frame,
  tf2_ros::Buffer& tf_buffer) {
  
  try {
      // Get the transform from camera frame to base frame
      // Note: We need to transform FROM camera TO base
      geometry_msgs::TransformStamped transform_stamped = 
          tf_buffer.lookupTransform("panda_link0", "depth", 
                                   ros::Time(0), ros::Duration(1.0));
      
      // Create a point in geometry_msgs format
      geometry_msgs::PointStamped point_cam;
      point_cam.header.frame_id = "depth";
      point_cam.header.stamp = ros::Time(0);
      point_cam.point.x = point_camera_frame[0];
      point_cam.point.y = point_camera_frame[1];
      point_cam.point.z = point_camera_frame[2];
      
      // Transform the point
      geometry_msgs::PointStamped point_base;
      tf2::doTransform(point_cam, point_base, transform_stamped);
      
      // Return as vector
      return {static_cast<float>(point_base.point.x),
              static_cast<float>(point_base.point.y),
              static_cast<float>(point_base.point.z)};
  }
  catch (tf2::TransformException &ex) {
      ROS_ERROR("Failed to transform point from camera to base: %s", ex.what());
      // Return original point as fallback
      return point_camera_frame;
  }
}

// Integrated function that combines shape classification and size estimation
std::tuple<std::string, int, std::vector<float>, float> classifyAndMeasureShape(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud) {
  if (cloud->empty()) {
      return std::make_tuple("none", -1, std::vector<float>{0.0f, 0.0f, 0.0f}, 0.0f);
  }

  // Set up TF buffer and listener.
static tf2_ros::Buffer tf_buffer;
static tf2_ros::TransformListener tf_listener(tf_buffer);


  // Step 1: Find the centroid of the point cloud
  Eigen::Vector4f centroid;
  pcl::compute3DCentroid(*cloud, centroid);
  
  // Step 2: Perform PCA to find principal axes
  pcl::PCA<pcl::PointXYZRGB> pca;
  pca.setInputCloud(cloud);
  Eigen::Matrix3f eigenVectors = pca.getEigenVectors();
  
  // Calculate rotation angle from the principal axis
  Eigen::Vector3f mainAxis(eigenVectors(0, 0), eigenVectors(1, 0), eigenVectors(2, 0));
  float rotationAngle = atan2(mainAxis(1), mainAxis(0));
  
  // Step 3: Create a normalized point cloud by aligning it with axes
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr normalizedCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
  normalizedCloud->points.resize(cloud->points.size());
  
  float cosTheta = cos(rotationAngle);
  float sinTheta = sin(rotationAngle);
  
  for (size_t i = 0; i < cloud->points.size(); i++) {
      float x = cloud->points[i].x - centroid[0];
      float y = cloud->points[i].y - centroid[1];
      
      normalizedCloud->points[i].x = x * cosTheta - y * sinTheta;
      normalizedCloud->points[i].y = x * sinTheta + y * cosTheta;
      normalizedCloud->points[i].z = cloud->points[i].z - centroid[2];
      normalizedCloud->points[i].rgb = cloud->points[i].rgb;
  }
  
  // Get the bounding box of the normalized cloud
  float minX = std::numeric_limits<float>::max();
  float minY = std::numeric_limits<float>::max();
  float maxX = -std::numeric_limits<float>::max();
  float maxY = -std::numeric_limits<float>::max();
  
  for (const auto& point : normalizedCloud->points) {
      minX = std::min(minX, point.x);
      minY = std::min(minY, point.y);
      maxX = std::max(maxX, point.x);
      maxY = std::max(maxY, point.y);
  }

  float width = maxX - minX;
  float height = maxY - minY; 
  float avgDimension = (width + height) / 2.0f;
  float aspectRatio = width / height;
  
  // Step 4: Create concentric circle analysis
  const int numCircles = 5;
  std::vector<int> circlePointCount(numCircles, 0);
  std::vector<float> circleDensities(numCircles, 0.0f);
  
  float maxRadius = std::min(width, height) / 2.0f;
  float centerX = (maxX + minX) / 2.0f;
  float centerY = (maxY + minY) / 2.0f;
  
  // Count points in each concentric circle
  for (const auto& point : normalizedCloud->points) {
      float dx = point.x - centerX;
      float dy = point.y - centerY;
      float distance = std::sqrt(dx*dx + dy*dy);
      float normalizedDistance = distance / maxRadius;
      
      int circleIndex = std::min(static_cast<int>(normalizedDistance * numCircles), numCircles - 1);
      circlePointCount[circleIndex]++;
  }
  
  // Calculate actual areas of each ring
  std::vector<float> ringAreas(numCircles);
  for (int i = 0; i < numCircles; i++) {
      float outerRadius = maxRadius * (i + 1) / numCircles;
      float innerRadius = (i == 0) ? 0 : maxRadius * i / numCircles;
      ringAreas[i] = M_PI * (outerRadius * outerRadius - innerRadius * innerRadius);
  }
  
  // Normalize by area to get density
  float totalArea = M_PI * maxRadius * maxRadius;
  float totalPoints = normalizedCloud->points.size();
  float averagePointDensity = totalPoints / totalArea;
  
  for (int i = 0; i < numCircles; i++) {
      float expectedPoints = averagePointDensity * ringAreas[i];
      if (expectedPoints > 0) {
          circleDensities[i] = static_cast<float>(circlePointCount[i]) / expectedPoints;
      }
  }
  
  // Step 5: Calculate angular distribution
  const int numAngles = 36;
  std::vector<int> angularCounts(numAngles, 0);
  
  for (const auto& point : normalizedCloud->points) {
      float dx = point.x - centerX;
      float dy = point.y - centerY;
      float angle = atan2(dy, dx);
      
      if (angle < 0) angle += 2 * M_PI;
      int angleBin = static_cast<int>((angle / (2 * M_PI)) * numAngles) % numAngles;
      angularCounts[angleBin]++;
  }
  
  // Calculate normalized angular counts and variance
  std::vector<float> normalizedAngularCounts(numAngles);
  float angularSum = 0.0f;
  
  for (int i = 0; i < numAngles; i++) {
      normalizedAngularCounts[i] = static_cast<float>(angularCounts[i]) / totalPoints;
      angularSum += normalizedAngularCounts[i];
  }
  
  float angularMean = angularSum / numAngles;
  float angularVariance = 0.0f;
  
  for (float count : normalizedAngularCounts) {
      angularVariance += (count - angularMean) * (count - angularMean);
  }
  angularVariance /= numAngles;
  
  // Step 6: Inner/outer ratio analysis
  float innerRadius = maxRadius * 0.3f;
  int innerPoints = 0;
  int outerPoints = 0;
  
  for (const auto& point : normalizedCloud->points) {
      float dx = point.x - centerX;
      float dy = point.y - centerY;
      float distance = std::sqrt(dx*dx + dy*dy);
      
      if (distance < innerRadius) {
          innerPoints++;
      } else {
          outerPoints++;
      }
  }
  
  float innerRatio = static_cast<float>(innerPoints) / totalPoints;
  float outerRatio = static_cast<float>(outerPoints) / totalPoints;
  
  // Step 7: Quadrant analysis
  int q1Points = 0, q2Points = 0, q3Points = 0, q4Points = 0;
  
  for (const auto& point : normalizedCloud->points) {
      float dx = point.x - centerX;
      float dy = point.y - centerY;
      
      if (dx >= 0 && dy >= 0) q1Points++;
      else if (dx < 0 && dy >= 0) q2Points++;
      else if (dx < 0 && dy < 0) q3Points++;
      else q4Points++;
  }
  
  std::vector<float> quadrantRatios = {
      static_cast<float>(q1Points) / totalPoints,
      static_cast<float>(q2Points) / totalPoints,
      static_cast<float>(q3Points) / totalPoints,
      static_cast<float>(q4Points) / totalPoints
  };
  
  float quadrantMean = 0.25f;
  float quadrantVariance = 0.0f;
  
  for (float ratio : quadrantRatios) {
      quadrantVariance += (ratio - quadrantMean) * (ratio - quadrantMean);
  }
  quadrantVariance /= 4;
  
  // Debug output
  ROS_INFO("Shape and size metrics:");
  ROS_INFO("  Total points: %lu", cloud->points.size());
  ROS_INFO("  Height: %.2f, Width: %.2f", height, width);
  ROS_INFO("  Aspect ratio: %.2f", aspectRatio);
  ROS_INFO("  Rotation angle: %.2f degrees", rotationAngle * 180 / M_PI);
  // ROS_INFO("  Circle densities: [%.2f, %.2f, %.2f, %.2f, %.2f]", 
  //          circleDensities[0], circleDensities[1], circleDensities[2], 
  //          circleDensities[3], circleDensities[4]);
  // ROS_INFO("  Angular variance: %.5f", angularVariance);
  ROS_INFO("  Inner/Outer ratio: %.2f/%.2f", innerRatio, outerRatio);
  ROS_INFO("  Average dimension: %.2f", avgDimension);

  
  // Shape classification
  std::string shape = "none";
  
  // NOUGHT DETECTION
  bool isNought = false;
  
  if (circleDensities[0] <= 0.1 && 
      circleDensities[4] >= 0.3 &&
      circleDensities[4] > circleDensities[0] * 3 &&
      angularVariance < 0.0001 &&
      innerRatio < 0.1 &&
      aspectRatio > 0.9 && aspectRatio < 1.1) {
      isNought = true;
  }

  // Alternative nought detection
  if (innerRatio < 0.01 &&
    aspectRatio > 0.95 && aspectRatio < 1.05){
      isNought = true;
  }
  
  // CROSS DETECTION
  bool isCross = false;
  
  if (innerRatio > 0.1 &&
      std::abs(circleDensities[0] - circleDensities[4]) > 0.2 &&
      angularVariance > 0.0001 &&
      aspectRatio > 0.9 && aspectRatio < 1.1) {
      isCross = true;
  }
  
  // Alternative cross detection
  if (innerRatio > outerRatio * 0.5 &&
      quadrantVariance < 0.05) {
      isCross = true;
  }
  
  ROS_INFO("  Classification results: isNought=%d, isCross=%d", isNought, isCross);
  
  if (isNought) {
      shape = "nought";
  } else if (isCross) {
      shape = "cross";
  }
  
  // Size determination based on the average dimension
  int size = -1;
  
  if (avgDimension >= 0.07f && avgDimension <= 0.125f) {
      size = 20;
  } else if (avgDimension > 0.125f && avgDimension <= 0.175f) {
      size = 30;
  } else if (avgDimension > 0.175f && avgDimension <= 0.225f) {
      size = 40;
  }
  
  // Return shape classification, size, and centroid
  std::vector<float> centroidVec = {centroid[0], centroid[1], centroid[2]};

  std::vector<float> transformedCentroid = transformPointCameraToBase(centroidVec, tf_buffer);
  centroidVec[0] = transformedCentroid[0];
  centroidVec[1] = transformedCentroid[1];
  centroidVec[2] = transformedCentroid[2];

  ROS_INFO("Final shape: %s, size: %d, centroid: [%.3f, %.3f]", 
           shape.c_str(), size, centroidVec[0], centroidVec[1]);
  return std::make_tuple(shape, size, centroidVec, rotationAngle);
}



  bool isNewCentroid(const geometry_msgs::Point &worldCentroid,
    const std::vector<geometry_msgs::Point>& existingCentroids,
    double threshold = 0.05) {
    double thresholdSquared = threshold * threshold;
    for (const auto &existing : existingCentroids) {
      double dx = worldCentroid.x - existing.x;
      double dy = worldCentroid.y - existing.y;
      double dz = worldCentroid.z - existing.z;
      double distanceSquared = dx * dx + dy * dy + dz * dz;
    if (distanceSquared < thresholdSquared) {
      return false;
    }
    }
    return true;
  }

  bool solve(const cw2_world_spawner::Task3Service::Request &req,
    cw2_world_spawner::Task3Service::Response &res,
    cw2 &robot, ros::NodeHandle &nh) {
ROS_INFO("[Task3] Solving Task 3...");

// Set up TF buffer and listener.
static tf2_ros::Buffer tf_buffer;
static tf2_ros::TransformListener tf_listener(tf_buffer);

// Declare containers for storing centroids, counts, and clouds.
std::vector<geometry_msgs::Point> worldCentroidsNought;
std::vector<geometry_msgs::Point> worldCentroidsCross;
std::map<std::string, int> count;
std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> shapePointClouds;

// Vector to store detailed detection info.
std::vector<std::string> detection_info;

// Create a vector to hold the scan poses.
std::vector<geometry_msgs::Pose> scan_poses;

// Set up the common orientation and z-height.
tf2::Quaternion quat;
quat.setRPY(M_PI, 0, -M_PI / 4);
geometry_msgs::Pose pose;
pose.orientation = tf2::toMsg(quat);
pose.position.z = 0.75;

// Define scan poses.
std::vector<double> x_values = {0.4, 0.4, 0.4, 0, -0.4, -0.4, -0.4, 0};
std::vector<double> y_values = {-0.4, 0, 0.4, 0.4, 0.4, 0, -0.4, -0.4};

for (size_t i = 0; i < x_values.size(); ++i) {
pose.position.x = x_values[i];
pose.position.y = y_values[i];
scan_poses.push_back(pose);
}

// Iterate over all the scan poses.
for (const auto &scan_pose : scan_poses) {
if (robot.moveArm(scan_pose)) {
   ros::Duration(1.0).sleep();

   auto cloud = capturePointCloud(nh);
   if (!cloud || cloud->empty() || cloud->points.size() > 300000)
       continue;

   auto clusters = extractClusters(cloud);
   if (clusters.empty()) 
       continue;
   
   for (const auto &cluster : clusters) {
       auto shapeTuple = classifyAndMeasureShape(cluster);
        std::string shape = std::get<0>(shapeTuple);
        int estimatedSize = std::get<1>(shapeTuple);
        std::vector<float> centroid = std::get<2>(shapeTuple);
        float rotation_angle = std::get<3>(shapeTuple);

       if (shape == "none") continue;
       // Estimate size and centroid.
       if (estimatedSize == -1) continue; // Skip if size is unknown.

       geometry_msgs::Point worldCentroid;
       worldCentroid.x = centroid[0];
       worldCentroid.y = centroid[1];
       worldCentroid.z = centroid[2];

       bool isNew = false;
       if (shape == "nought") {  // replaced "square" with "nought"
           isNew = isNewCentroid(worldCentroid, worldCentroidsNought);
           if (isNew)
               worldCentroidsNought.push_back(worldCentroid);
       } else if (shape == "cross") {
           isNew = isNewCentroid(worldCentroid, worldCentroidsCross);
           if (isNew)
               worldCentroidsCross.push_back(worldCentroid);
       }

       if (isNew) {
           count[shape]++;
           shapePointClouds.push_back(cloud);
       }

       // Save detailed detection info.
       char info[200];
       snprintf(info, sizeof(info), "Detected shape: %s (size: %d) at (%.2f, %.2f)",
                shape.c_str(), estimatedSize, worldCentroid.x, worldCentroid.y);
       detection_info.push_back(std::string(info));
       // Print the detection info.
       ROS_INFO("%s", info);
   } // end of clusters loop

} // end of robot.moveArm
} // end of scan_poses loop

ROS_INFO("Final Detections:");
for (const auto &info : detection_info) {
    ROS_INFO("%s", info.c_str());
}
ROS_INFO("Final Counts - Noughts: %d, Crosses: %d", count["nought"], count["cross"]);

float total_shapes = count["nought"] + count["cross"];
float score = 0.0f;
if (count["nought"] > count["cross"]){
  score = count["nought"];
}
else if (count["cross"] > count["nought"]){
  score = count["cross"];
}
else if (count["cross"] == count["nought"]){
  score = count["cross"];
}
else{
  score = 0;
}

return true;
} // end solve

} // namespace task3
