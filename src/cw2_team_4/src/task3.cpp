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

    std::vector<cv::Point2f> centroidsCross;
    std::vector<cv::Point2f> centroidsSquare;
    std::map<std::string, int> count;

    std::string classifyShape(const std::vector<cv::Point> &contour) {
        if (contour.empty()) {
            return "none";
        }
        
        std::vector<cv::Point> approx;
        cv::approxPolyDP(contour, approx, 0.02 * cv::arcLength(contour, true), true);
        
        if (approx.size() == 4 && cv::isContourConvex(approx)) {
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
        float &scale, float &offsetX, float &offsetY) {
      float minX = std::numeric_limits<float>::max();
      float maxX = -std::numeric_limits<float>::max();
      float minY = std::numeric_limits<float>::max();
      float maxY = -std::numeric_limits<float>::max();
    
      for (const auto &pt : cloud->points) {
          if (pt.x < minX) minX = pt.x;
          if (pt.x > maxX) maxX = pt.x;
          if (pt.y < minY) minY = pt.y;
          if (pt.y > maxY) maxY = pt.y;
      }
    
      float rangeX = maxX - minX;
      float rangeY = maxY - minY;
      scale = std::min(imageWidth / rangeX, imageHeight / rangeY) * 0.8;
    
      offsetX = -minX * scale;
      offsetY = -minY * scale;
    
      cv::Mat image = cv::Mat::zeros(imageHeight, imageWidth, CV_8UC1);
      for (const auto &pt : cloud->points) {
          int x = static_cast<int>(pt.x * scale + offsetX);
          int y = static_cast<int>(pt.y * scale + offsetY);
          if (x >= 0 && x < imageWidth && y >= 0 && y < imageHeight) {
              image.at<uchar>(y, x) = 255;
          }
      }
      return image;
    }

    cv::Point2f computeCentroid(const std::vector<cv::Point> &contour) {
        cv::Moments moments = cv::moments(contour, false);
        return cv::Point2f(moments.m10 / moments.m00, moments.m01 / moments.m00);
    }

    bool isNewCentroid(const cv::Point2f &centroid, double threshold = 15.0) {
        for (const auto &existing : centroidsSquare) {
            if (cv::norm(centroid - existing) < threshold) {
                return false;
            }
        }
        for (const auto &existing : centroidsCross) {
            if (cv::norm(centroid - existing) < threshold) {
                return false;
            }
        }
        return true;
    }

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

        for (const auto& point : cloud->points) {
            // Extract RGB values from the point
            int r = static_cast<int>(point.r);
            int g = static_cast<int>(point.g);
            int b = static_cast<int>(point.b);
    
            // Check if the point color matches one of the three target colors
            if ((r == 26 && g == 26 && b == 204) ||   // Blue
                (r == 204 && g == 26 && b == 204) ||  // Magenta (Purple-Pink)
                (r == 204 && g == 26 && b == 26)) {   // Red
                filtered_cloud->points.push_back(point);
            }
        }
    
        filtered_cloud->width = filtered_cloud->points.size();
        filtered_cloud->height = 1;
        filtered_cloud->is_dense = true;
    
        // Replace the original cloud with the filtered cloud
        cloud->swap(*filtered_cloud);

    if (filtered_cloud->empty()) {
        ROS_WARN("Color filtering removed all points; using original cloud.");
        return cloud;
    } else {
        ROS_INFO("After color filtering, %lu points remain.", filtered_cloud->points.size());
        return filtered_cloud;
    }
}


    std::vector<cv::Point> extractContour(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud) {
        float scale, offsetX, offsetY;
        cv::Mat img = pointCloudToImage(cloud, 500, 500, scale, offsetX, offsetY);
        cv::GaussianBlur(img, img, cv::Size(5,5), 0);
        
        cv::Mat edges;
        cv::Canny(img, edges, 50, 150);
        
        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(edges, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
        
        if (contours.empty()) {
            return {};
        }
        
        double maxArea = 0;
        int maxIdx = 0;
        for (size_t i = 0; i < contours.size(); i++) {
            double area = cv::contourArea(contours[i]);
            if (area > maxArea) {
                maxArea = area;
                maxIdx = i;
            }
        }
        return contours[maxIdx];
    }

    bool solve(const cw2_world_spawner::Task3Service::Request &req,
               cw2_world_spawner::Task3Service::Response &res, cw2& robot, ros::NodeHandle &nh) {
        ROS_INFO("[Task3] Solving Task 3...");
        
        double x_min = -0.6, x_max = 0.7;
        double y_min = -0.55, y_max = 0.55;
        double step_size = 0.2;
        
        geometry_msgs::Pose scan_pose;
        scan_pose.orientation.w = 1.0;
        scan_pose.position.z = 0.5;
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
                    if (contour.empty()) continue;
                    
                    std::string shape = classifyShape(contour);
                    cv::Point2f centroid = computeCentroid(contour);
                    
                    if (isNewCentroid(centroid)) {
                        if (shape == "square") {
                            centroidsSquare.push_back(centroid);
                            count["square"]++;
                        } else if (shape == "cross") {
                            centroidsCross.push_back(centroid);
                            count["cross"]++;
                        }
                    }
                    
                    ROS_INFO("Detected shape: %s at (%.2f, %.2f)", shape.c_str(), centroid.x, centroid.y);
                }
            }
        }
        std::cout<<"*********************************************"<<std::endl;
        std::cout<<"The centroids are as follows:  "<<std::endl;
        std::cout<<"*********************************************"<<std::endl;
        for (const auto& centroid : centroidsSquare) {
            std::cout<<"Square: ("<<centroid.x<<", "<<centroid.y<<")"<<std::endl;
        }
        for (const auto& centroid : centroidsCross) {
            std::cout<<"Cross: ("<<centroid.x<<", "<<centroid.y<<")"<<std::endl;
        }
        
        ROS_INFO("Final Counts - Squares: %d, Crosses: %d", count["square"], count["cross"]);
        return true;
    }
}
