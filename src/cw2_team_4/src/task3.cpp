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
    const int green_threshold = 50;
    // const int threshold = 100; // Adjust based on your sensor's scaling.
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

  std::vector<cv::Point> getIdealContour(const std::string &shapeType,
    float scale, float offsetX, float offsetY)
  {
  std::vector<cv::Point> ideal;
  if (shapeType == "cross") {
    // Create a binary image.
    int imgSize = 400;  // Adjust image size as needed.
    cv::Mat img = cv::Mat::zeros(imgSize, imgSize, CV_8UC1);
    int thickness = 10;
    cv::Point center(imgSize / 2, imgSize / 2);
    int lineLength = imgSize / 2 - 50;  // Adjust length as needed.

    // Draw a plus sign ("+") on the image:
    // Vertical line.
    cv::line(img, cv::Point(center.x, center.y - lineLength),
                  cv::Point(center.x, center.y + lineLength),
                  cv::Scalar(255), thickness);
    // Horizontal line.
    cv::line(img, cv::Point(center.x - lineLength, center.y),
                  cv::Point(center.x + lineLength, center.y),
                  cv::Scalar(255), thickness);

    // Extract contours from the binary image.
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(img, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    if (!contours.empty()) {
        // Find the largest contour by area.
        size_t largestContourIdx = 0;
        double maxArea = 0;
        for (size_t i = 0; i < contours.size(); i++) {
            double area = cv::contourArea(contours[i]);
            if (area > maxArea) {
                maxArea = area;
                largestContourIdx = i;
            }
        }
        // Scale and offset each point from the ideal contour.
        for (const auto &pt : contours[largestContourIdx]) {
            int x = static_cast<int>(pt.x * scale + offsetX);
            int y = static_cast<int>(pt.y * scale + offsetY);
            ideal.push_back(cv::Point(x, y));
        }
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

  std::string classifyShape(const std::vector<cv::Point> &contour, float scale, float offsetX, float offsetY) {
      if (contour.empty()) {
          ROS_WARN("No contour found for shape classification.");
          return "none";
      }
      // Compute the shape match metric for debugging.
      std::vector<cv::Point> idealCrossContour = getIdealContour("cross", scale, offsetX, offsetY);
      std::vector<cv::Point> idealNoughtContour = getIdealContour("nought", scale, offsetX, offsetY);
      double matchCross = cv::matchShapes(contour, idealCrossContour, cv::CONTOURS_MATCH_I1, 0.0);
      double matchNought = cv::matchShapes(contour, idealNoughtContour, cv::CONTOURS_MATCH_I1, 0.0);
      ROS_INFO("Shape match cross value: %f", matchCross);
      ROS_INFO("Shape match nought value: %f", matchNought);

      if (matchCross < matchNought && matchCross < 0.1) {
          return "cross";
      } 
      else if(matchNought < matchCross && matchNought < 0.1) {
          return "nought";
      }
      else {
          return "none";
      }
  }

    
  std::pair<int, std::vector<float>> estimateSize(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud) {
    float minX = std::numeric_limits<float>::max();
    float maxX = -std::numeric_limits<float>::max();
    float minY = std::numeric_limits<float>::max();
    float maxY = -std::numeric_limits<float>::max();
    float sumX = 0.0f, sumY = 0.0f, sumZ = 0.0f;
    int count = 0;

    // Compute bounding box (x-y only) and sums for centroid calculation.
    for (const auto &pt : cloud->points) {
        if (pt.x < minX) minX = pt.x;
        if (pt.x > maxX) maxX = pt.x;
        if (pt.y < minY) minY = pt.y;
        if (pt.y > maxY) maxY = pt.y;
        
        sumX += pt.x;
        sumY += pt.y;
        sumZ += pt.z;
        ++count;
    }
    
    // Compute the centroid as the average of all points.
    std::vector<float> centroid(3, 0.0f);
    if (count > 0) {
        centroid[0] = sumX / count;
        centroid[1] = sumY / count;
        centroid[2] = sumZ / count;
    }
    
    float rangeX = maxX - minX;
    float rangeY = maxY - minY;
    
    int size = -1; // Default: unknown size
    // Use proper chained comparisons in C++
    if (rangeX >= 0.08f && rangeX <= 0.12f && rangeY >= 0.08f && rangeY <= 0.12f) {
        size = 20;
    } else if (rangeX > 0.13f && rangeX <= 0.17f && rangeY > 0.13f && rangeY <= 0.17f) {
        size = 30;
    } else if (rangeX > 0.18f && rangeX <= 0.22f && rangeY > 0.18f && rangeY <= 0.22f) {
        size = 40;
    }
    
    return std::make_pair(size, centroid);
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

  // Declare containers for storing centroids, counts, and clouds.
  std::vector<geometry_msgs::Point> worldCentroidsNought;
  std::vector<geometry_msgs::Point> worldCentroidsCross;
  std::map<std::string, int> count;
  std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> shapePointClouds;

  // Create a vector to hold the scan poses.
  std::vector<geometry_msgs::Pose> scan_poses;

  // Set up the common orientation and z-height.
  tf2::Quaternion quat;
  quat.setRPY(M_PI, 0, -M_PI / 4);
  geometry_msgs::Pose pose;
  pose.orientation = tf2::toMsg(quat);
  pose.position.z = 0.55;

  std::vector<double> x_values = {-0.45, -0.25, 0, 0.25, 0.55, 0.55, 0.55, 0.55, 0.25, 0.35, 0.35, -0.35, -0.5, -0.5, -0.35, -0.45, -0.25, 0};
  std::vector<double> y_values = {-0.4, -0.4, -0.4, -0.4, -0.4, -0.2, 0.2, 0.4, 0.4, 0.2, -0.2, -0.2, -0.2, 0.2, 0.2, 0.4, 0.4, 0.4};

  for (size_t i = 0; i < x_values.size(); ++i) {
          pose.position.x = x_values[i];
          pose.position.y = y_values[i];
          scan_poses.push_back(pose);
  }


// Now you can iterate over these 8 scan poses.
for (const auto &scan_pose : scan_poses) {

     if (robot.moveArm(scan_pose)) {
         ros::Duration(1.0).sleep();

         auto cloud = capturePointCloud(nh);
         if (!cloud || cloud->empty()) continue;

         auto contour = extractContour(cloud);
         if (contour.empty()) continue;

         float scale, offsetX, offsetY;
         cv::Mat img = pointCloudToImage(cloud, 500, 500, scale, offsetX, offsetY);
         std::string shape = classifyShape(contour, scale, offsetX, offsetY);
         // Use estimateSize which returns a pair<int, vector<float>>
         auto sizePair = estimateSize(cloud);
         int estimatedSize = sizePair.first;

         std::vector<float> centroid = sizePair.second;
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
             // Removed marker publishing call.
         }

         ROS_INFO("Detected shape: %s (size: %d) at (%.2f, %.2f)",
                  shape.c_str(), estimatedSize, worldCentroid.x, worldCentroid.y);
     }
 }


ROS_INFO("Final Counts - Noughts: %d, Crosses: %d", count["nought"], count["cross"]);
return true;
}

} // namespace task3