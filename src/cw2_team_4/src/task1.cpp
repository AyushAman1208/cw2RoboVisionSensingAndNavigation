#include "cw2_class.h"
#include "task1.h"

namespace task1 {
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr capturePointCloud(ros::NodeHandle &nh) { 
    ROS_INFO("Waiting for a fresh point cloud...");

    std::string topic = "/r200/camera/depth_registered/points";
    const double freshnessThreshold = 10.0; // Accept only messages published in the past 10 seconds
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

    // Filter points by color: remove ground points (assumed to be green).
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    const int green_threshold = 50; // Adjust based on your sensor's scaling.
    for (const auto &pt : cloud->points) {
        if (pt.g < green_threshold) {  
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

  // Convert point cloud to image using dynamic scaling.
  cv::Mat pointCloudToImage(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud,
    int imageWidth, int imageHeight,
    float &scale, float &offsetX, float &offsetY)
  {
  float minX = std::numeric_limits<float>::max();
  float maxX = -std::numeric_limits<float>::max();
  float minY = std::numeric_limits<float>::max();
  float maxY = -std::numeric_limits<float>::max();

  // Compute the bounding box of the cloud (x-y only)
  for (const auto &pt : cloud->points) {
  if (pt.x < minX) minX = pt.x;
  if (pt.x > maxX) maxX = pt.x;
  if (pt.y < minY) minY = pt.y;
  if (pt.y > maxY) maxY = pt.y;
  }

  float rangeX = maxX - minX;
  float rangeY = maxY - minY;
  scale = std::min(imageWidth / rangeX, imageHeight / rangeY);
  offsetX = -minX * scale;  // This will map minX to 0.
  offsetY = -minY * scale;  // This will map minY to 0.

  cv::Mat image = cv::Mat::zeros(imageHeight, imageWidth, CV_8UC1);
  for (const auto &pt : cloud->points) {
  int x = static_cast<int>(pt.x * scale + offsetX);
  int y = static_cast<int>(pt.y * scale + offsetY);
  if (x >= 0 && x < imageWidth && y >= 0 && y < imageHeight) {
  image.at<uchar>(y, x) = 255; // mark point as white
  }
  }
  return image;
  }

  std::vector<cv::Point> getIdealContour(const std::string &shapeType,
    float scale, float offsetX, float offsetY)
  {
  std::vector<cv::Point> ideal;
  if (shapeType == "cross") {
  // Ideal points for a cross
  // Edges of the cross
  std::vector<cv::Point> temp = {
  cv::Point(100, 20), cv::Point(100, -20),
  cv::Point(-100, 20), cv::Point(-100, -20),
  cv::Point(20, 100), cv::Point(20, -100),
  cv::Point(-20, 100), cv::Point(-20, -100)
  };
  for (const auto &pt : temp) {
  int x = static_cast<int>(pt.x * scale + offsetX);
  int y = static_cast<int>(pt.y * scale + offsetY);
  ideal.push_back(cv::Point(x, y));
  }
  } else if (shapeType == "nought") {
  // Ideal points for a square (nought)
  // Inner and Outer corners of the square
  std::vector<cv::Point> temp = {
  cv::Point(100, 100), cv::Point(100, -100),
  cv::Point(-100, -100), cv::Point(-100, 100),
  cv::Point(60, 60),   cv::Point(60, -60),
  cv::Point(-60, -60), cv::Point(-60, 60)
  };
  for (const auto &pt : temp) {
  int x = static_cast<int>(pt.x * scale + offsetX);
  int y = static_cast<int>(pt.y * scale + offsetY);
  ideal.push_back(cv::Point(x, y));
  }
  } else {
  ROS_ERROR("Unknown shape type: %s", shapeType.c_str());
  }
  return ideal;
  }

  // Compute orientation from the observed contour using minAreaRect and shape matching.
float computeOrientationCV(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud,
  const std::string &shapeType)
{
  int imageWidth = 500, imageHeight = 500;
  float scale, offsetX, offsetY;
  // Convert cloud to image using dynamic scaling.
  cv::Mat img = pointCloudToImage(cloud, imageWidth, imageHeight, scale, offsetX, offsetY);

  // Optional: Display the image for debugging.

  // cv::imshow("Original Image", img);
  // cv::waitKey(0);

// Apply Gaussian blur to remove noise.
cv::GaussianBlur(img, img, cv::Size(5,5), 0);

// Use Canny edge detection.
cv::Mat edges;
cv::Canny(img, edges, 50, 150);

// Find contours in the edge-detected image.
std::vector<std::vector<cv::Point>> contours;
cv::findContours(edges, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
if (contours.empty()) {
ROS_WARN("No contours found in image; defaulting orientation to 0.");
return 0.0f;
}

// Find the largest contour.
double maxArea = 0.0;
int maxIdx = 0;
for(size_t i = 0; i < contours.size(); i++){
    double area = cv::contourArea(contours[i]);
    if(area > maxArea){
        maxArea = area;
        maxIdx = i;
    }
}
std::vector<cv::Point> contourObserved = contours[maxIdx];
if (contourObserved.empty()) {
ROS_WARN("No contour found in image; defaulting orientation to 0.");
return 0.0f;
}

// Compute the shape match metric for debugging.
std::vector<cv::Point> idealContour = getIdealContour(shapeType,scale, offsetX, offsetY);
double matchValue = cv::matchShapes(contourObserved, idealContour, cv::CONTOURS_MATCH_I1, 0.0);
ROS_INFO("Shape match value: %f", matchValue);

// Draw the contour and its minAreaRect on a copy of the original image for visualization.
cv::Mat imageColor;
cv::cvtColor(img, imageColor, cv::COLOR_GRAY2BGR);
cv::drawContours(imageColor, contours, maxIdx, cv::Scalar(0,255,0), 2);

// Get orientation from minAreaRect.
cv::RotatedRect rect = cv::minAreaRect(contourObserved);
cv::Point2f rectPoints[4];
rect.points(rectPoints);
for (int j = 0; j < 4; j++) {
    cv::line(imageColor, rectPoints[j], rectPoints[(j+1)%4], cv::Scalar(0,0,255), 2);
}
// Optional: Draw the ideal contour for debugging.
// cv::imshow("Detected Orientation", imageColor);
// cv::waitKey(0);

float observedAngle = rect.angle; // in degrees, range [-90, 0)
ROS_INFO("Observed orientation from minAreaRect: %f degrees", observedAngle);


// Adjust the angle based on the shape type, determined by debugging ideal contours.
if (shapeType == "cross") {
    observedAngle += 45.0f;  // Adjust for cross shape
} else if (shapeType == "nought") {
    observedAngle += 0.0f;   // No adjustment for nought
}

float finalAngleDegrees = -45.0f - observedAngle;


// Convert the final angle (or the error) to radians:
float angleRad = finalAngleDegrees * (M_PI / 180.0f);

ROS_INFO("Final computed angle: %f degrees", finalAngleDegrees);

// For demonstration, return the computed angle in radians.
return angleRad;
}

  bool solve(const cw2_world_spawner::Task1Service::Request &req,
             cw2_world_spawner::Task1Service::Response &res)
  {
    ROS_INFO("[Task1] Solving Task 1...");

    // Create an instance of the cw2 class to access moveArm, moveGripper, etc.
    ros::NodeHandle nh;
    cw2 robot(nh);  // Pass the NodeHandle to the constructor

    // Move to home position.
    geometry_msgs::Pose home_pose;
    home_pose.position.x = 0.4;
    home_pose.position.y = 0.0;
    home_pose.position.z = 0.6; 

    tf2::Quaternion quat2;
    quat2.setRPY(M_PI, 0, -M_PI / 4);
    geometry_msgs::Quaternion original_orientation = tf2::toMsg(quat2);

    home_pose.orientation = original_orientation;  // Use the same orientation as the object
    if (!robot.moveArm(home_pose)) {
      ROS_ERROR("Failed to move back to home position");
      return false;
    }

    // Extract the object and goal points, and the shape type from the service request.
    geometry_msgs::PointStamped object_point = req.object_point;
    geometry_msgs::PointStamped goal_point   = req.goal_point;
    std::string shape_type = req.shape_type;

    // Move the robot arm to the object point.
    geometry_msgs::Pose observe_pose;
    observe_pose.position = object_point.point;
    observe_pose.position.z += 0.5;  // Adjust height for observing
    observe_pose.position.x -= 0.04;  // Adjust x position for observing

    observe_pose.orientation = original_orientation;

    if (!robot.moveArm(observe_pose)) {
      ROS_ERROR("Failed to move to observe pose");
      return false;
    }

    // Get the current point cloud.
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud;
    int attempts = 0;
    const int maxAttempts = 5; // Change maximum based on how long it takes to capture a point cloud, usually 1 or 2 attempts are enough.
    
    while (! (cloud = capturePointCloud(nh)) && attempts < maxAttempts) {
        ROS_WARN("Attempt %d: Failed to capture point cloud, retrying...", attempts+1);
        ros::Duration(1.0).sleep();  // wait 1 second before retrying
        attempts++;
    }
    
    if (!cloud) {
        ROS_ERROR("Failed to capture point cloud after %d attempts", attempts);
        return false;
    }

    // Compute the orientation of the object based on its point cloud and shape.
    float rotation_angle = computeOrientationCV(cloud, shape_type);
    
    // Use computed final angle to set the orientation of the arm.
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
    // Adjust the x and y positions based on the shape type and rotation angle.
    // This was determined by calculating the expected angles and the distances that the arm should move to be perpendicular to the object.
    
    if (shape_type == "nought") {
      if (rotation_angle > 0){
        object_pose.position.x += 0.08 * sin(rotation_angle + M_PI / 4);
        object_pose.position.y -= 0.08 * cos(rotation_angle + M_PI / 4);
      }
      else if (rotation_angle <= 0){
          object_pose.position.x -= 0.08 * sin(rotation_angle + M_PI / 4);
          object_pose.position.y += 0.08 * cos(rotation_angle + M_PI / 4);
      }
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

    
    // Move the arm back to home position.
    if (!robot.moveArm(home_pose)) {
      ROS_ERROR("Failed to move back to home position");
      return false;
    }
    // Shut down all windows
    cv::destroyAllWindows();
    cloud->clear();

    ROS_INFO("Task 1 completed successfully");
    return true;
  }

} // namespace task1

