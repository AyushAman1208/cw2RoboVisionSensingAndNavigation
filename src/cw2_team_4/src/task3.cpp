#include "task3.h"
#include "cw2_class.h"
#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <opencv2/opencv.hpp>

#include <pcl/visualization/pcl_visualizer.h>

namespace task3 {

    std::string classifyShape(const std::vector<cv::Point> &contour) {
        if (contour.empty()) {
            return "none";
        }
        
        std::vector<cv::Point> approx;
        cv::approxPolyDP(contour, approx, 0.02 * cv::arcLength(contour, true), true);
        
        if (approx.size() == 4) {
            return "square";
        }
        
        cv::Rect boundingBox = cv::boundingRect(contour);
        double aspectRatio = (double)boundingBox.width / boundingBox.height;
        
        if (aspectRatio > 0.8 && aspectRatio < 1.2) {
            return "cross";
        }
        
        return "none";
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

pcl::PointCloud<pcl::PointXYZRGB>::Ptr capturePointCloud(ros::NodeHandle &nh) {
    std::string topic = "/r200/camera/depth_registered/points";
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    auto msg = ros::topic::waitForMessage<sensor_msgs::PointCloud2>(topic, nh, ros::Duration(5.0));
    
    if (!msg) {
        ROS_WARN("No point cloud received.");
        return nullptr;
    }
    pcl::fromROSMsg(*msg, *cloud);

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
    return filtered_cloud;
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





bool solve(const cw2_world_spawner::Task3Service::Request &req,
           cw2_world_spawner::Task3Service::Response &res, cw2& robot, ros::NodeHandle &nh) {
    ROS_INFO("[Task3] Solving Task 3...");
    
    double x_min = -0.6, x_max = 0.7;
    double y_min = -0.55, y_max = 0.55;
    double step_size = 0.1;
    
    geometry_msgs::Pose scan_pose;
    scan_pose.orientation.w = 1.0;
    scan_pose.position.z = 0.6;
    tf2::Quaternion quat;
    quat.setRPY(M_PI, 0, -M_PI / 4);
    
    for (double y = y_min; y <= y_max; y += step_size) {
        for (double x = x_min; x <= x_max; x += step_size) {
            scan_pose.position.x = x;
            scan_pose.position.y = y;
            scan_pose.position.z = 0.6;
            scan_pose.orientation = tf2::toMsg(quat);
            bool armMoved = robot.moveArm(scan_pose);
            ros::Duration(1.0).sleep();
            
            if(armMoved){
                auto cloud = capturePointCloud(nh);
                if (!cloud || cloud->empty()) continue;
                    auto contour = extractContour(cloud);
                    std::string shape = classifyShape(contour);
                    std::cout<<"***************************"<<std::endl;
                    std::cout<<shape<<std::endl;
                    std::cout<<"***************************"<<std::endl;
                if (!contour.empty()) {
                    ROS_INFO("Contour extracted successfully at X=%.2f, Y=%.2f", x, y);
                }
            }
        }
    }
    return true;
}

}
