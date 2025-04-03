#include "cw2_class.h"
#include "task2.h"

namespace task2 {

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

// Function to take an observation at a given pose and return the point cloud.
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr takeObservation(geometry_msgs::PointStamped &observation_pose, ros::NodeHandle &nh, cw2 &robot) {
    geometry_msgs::Pose curr_pose;
    curr_pose.position.x = observation_pose.point.x;
    float yaw = 0.0;
    if (observation_pose.point.x < 0){
        curr_pose.position.x += 0.04;
        yaw = 3*M_PI / 4;
    }
    else if (observation_pose.point.x > 0){
        curr_pose.position.x -= 0.04;
        yaw = -M_PI / 4;
    }

    curr_pose.position.y = observation_pose.point.y;
    curr_pose.position.z = observation_pose.point.z + 0.5;
    tf2::Quaternion quat;
    quat.setRPY(M_PI, 0, yaw);
    curr_pose.orientation = tf2::toMsg(quat);
    
    if (!robot.moveArm(curr_pose)) return nullptr;
    ros::Duration(1.0).sleep();
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud = capturePointCloud(nh);
    if (!cloud || cloud->empty()) return nullptr;

    return cloud;
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
  scale = scale * 0.8; // Scale down to fit in the image

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

// Function to extract the largest contour from a point cloud image.
std::vector<cv::Point> extractContour(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud)
{
    // Convert point cloud to image (using your dynamic scaling function).
    float scale, offsetX, offsetY;
    cv::Mat img = pointCloudToImage(cloud, 500, 500, scale, offsetX, offsetY);

    // Optionally preprocess (e.g., Gaussian blur).
    cv::GaussianBlur(img, img, cv::Size(5,5), 0);

    // Use Canny edge detection.
    cv::Mat edges;
    cv::Canny(img, edges, 50, 150);

    // Find contours.
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(edges, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    if (contours.empty()) {
        ROS_WARN("No contours found!");
        return std::vector<cv::Point>();
    }

    // Return the largest contour.
    double maxArea = 0;
    int maxIdx = 0;
    for (size_t i = 0; i < contours.size(); i++) {
        double area = cv::contourArea(contours[i]);
        if (area > maxArea) {
            maxArea = area;
            maxIdx = i;
        }
    }
    // Optional: Show image for debugging
    cv::Mat imageColor;
    cv::cvtColor(img, imageColor, cv::COLOR_GRAY2BGR);
    cv::drawContours(imageColor, contours, maxIdx, cv::Scalar(0,255,0), 2);
    cv::imshow("Largest Contour", imageColor);
    cv::waitKey(0);

    return contours[maxIdx];
}

bool solve(const cw2_world_spawner::Task2Service::Request &req,
           cw2_world_spawner::Task2Service::Response &res, cw2& robot, ros::NodeHandle &nh)
{
  ROS_INFO("[Task2] Solving Task 2...");


  // Move to home position.
  geometry_msgs::Pose home_pose;
  home_pose.position.x = 0.4;
  home_pose.position.y = 0.0;
  home_pose.position.z = 0.6;
  tf2::Quaternion quat;
  quat.setRPY(M_PI, 0, -M_PI / 4);
  home_pose.orientation = tf2::toMsg(quat);
  if (!robot.moveArm(home_pose)) {
    ROS_ERROR("Failed to move back to home position");
    return false;
  }
  // Extract the reference points from the service request.
  std::vector<geometry_msgs::PointStamped> object_points = req.ref_object_points;
  geometry_msgs::PointStamped goal_point = req.mystery_object_point;

  // Take the observation position of each object and get point cloud
  auto ref_cloud1 = takeObservation(object_points.at(0), nh, robot);
  auto ref_cloud2 = takeObservation(object_points.at(1), nh, robot);
  auto goal_cloud = takeObservation(goal_point, nh, robot);

  // Extract the largest contour from each point cloud.
  std::vector<cv::Point> refContour1 = extractContour(ref_cloud1);
  std::vector<cv::Point> refContour2 = extractContour(ref_cloud2);
  std::vector<cv::Point> mysteryContour = extractContour(goal_cloud);
  if (refContour1.empty() || refContour2.empty() || mysteryContour.empty()) {
    ROS_ERROR("Failed to extract contours from point clouds");
    return false;
  }

  // Compare the mystery contour to each reference.
  double match1 = cv::matchShapes(mysteryContour, refContour1, cv::CONTOURS_MATCH_I1, 0.0);
  double match2 = cv::matchShapes(mysteryContour, refContour2, cv::CONTOURS_MATCH_I1, 0.0);

  ROS_INFO("Match score to reference 1: %f", match1);
  ROS_INFO("Match score to reference 2: %f", match2);

  if (match1 < match2) {
    res.mystery_object_num = 1;
    ROS_INFO("Mystery shape matches reference 1.");
  } else {
    res.mystery_object_num = 2;
    ROS_INFO("Mystery shape matches reference 2.");
  }

  return true;
}

}