#include "task3.h"
#include "cw2_class.h"
#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <opencv2/opencv.hpp>
#include <pcl/visualization/pcl_visualizer.h>
#include <visualization_msgs/Marker.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <cmath>

#include <sensor_msgs/PointCloud2.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <thread>
#include <chrono>

namespace task3 {

    std::vector<geometry_msgs::Point> worldCentroidsCross;
    std::vector<geometry_msgs::Point> worldCentroidsSquare;
    std::map<std::string, int> count;
    std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> shapePointClouds;

    std::string classifyShape(const std::vector<cv::Point> &contour) {
        if (contour.empty()) {
            return "none";
        }
    
        std::vector<cv::Point> approx;
        cv::approxPolyDP(contour, approx, 0.02 * cv::arcLength(contour, true), true);
    
        if (approx.size() == 4 && cv::isContourConvex(approx)) {
            // Check side lengths
            std::vector<double> sides;
            for (int i = 0; i < 4; ++i) {
                cv::Point pt1 = approx[i];
                cv::Point pt2 = approx[(i + 1) % 4];
                double length = cv::norm(pt1 - pt2);
                sides.push_back(length);
            }
    
            double avgSide = (sides[0] + sides[1] + sides[2] + sides[3]) / 4.0;
            bool sidesEqual = true;
            for (double side : sides) {
                if (std::abs(side - avgSide) > 0.2 * avgSide) {  // 20% tolerance
                    sidesEqual = false;
                    break;
                }
            }
    
            // Check angles (near 90°)
            bool rightAngles = true;
            for (int i = 0; i < 4; ++i) {
                cv::Point a = approx[i];
                cv::Point b = approx[(i + 1) % 4];
                cv::Point c = approx[(i + 2) % 4];
    
                cv::Point ab = b - a;
                cv::Point bc = c - b;
    
                double dot = ab.dot(bc);
                double mag1 = cv::norm(ab), mag2 = cv::norm(bc);
                double angle = std::acos(dot / (mag1 * mag2)) * 180.0 / CV_PI;
                std::cout << angle << std::endl;
                std::cout<<"Side equal: "<<sidesEqual<<std::endl;
    
                if (std::abs(angle - 90) > 20) {  // allow 20° tolerance
                    rightAngles = false;
                    break;
                }
            }
    
            if (sidesEqual && rightAngles) {
                return "square";
            }
        }
    
        // Consider "cross" if it is approximately square in bounding box
        cv::Rect boundingBox = cv::boundingRect(contour);
        double aspectRatio = static_cast<double>(boundingBox.width) / boundingBox.height;
        if (aspectRatio > 0.8 && aspectRatio < 1.2) {
            return "cross";
        }
    
        return "none";
    }
    

    std::string estimateShapeSize(const std::vector<cv::Point> &contour, float scale) {
        float area = static_cast<float>(cv::contourArea(contour));
        float realArea = area / (scale * scale);

        if (realArea < 11000) return "20mm";
        else if (realArea < 22000) return "30mm";
        else return "40mm";
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

    bool isNewCentroid(const geometry_msgs::Point &worldCentroid, const std::vector<geometry_msgs::Point>& existingCentroids, double threshold = 0.05) {
        for (const auto &existing : existingCentroids) {
            double dx = worldCentroid.x - existing.x;
            double dy = worldCentroid.y - existing.y;
            double dz = worldCentroid.z - existing.z;
            double distance = std::sqrt(dx * dx + dy * dy + dz * dz);
            if (distance < threshold) {
                return false;
            }
        }
        return true;
    }


    
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr capturePointCloud(ros::NodeHandle &nh) { 
        ROS_INFO("Waiting for a fresh point cloud...");

        std::string topic = "/r200/camera/depth_registered/points";
        const double freshnessThreshold = 3.0;
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
        bool received = false;

        int attempts = 0;
        const int maxAttempts = 10;
        while (!received && attempts < maxAttempts) {
            auto msg = ros::topic::waitForMessage<sensor_msgs::PointCloud2>(topic, nh, ros::Duration(5.0));
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


    

    void publishMarker(const geometry_msgs::Point& position, const std::string& id_str, const std::string& shape, ros::Publisher& marker_pub) {
        visualization_msgs::Marker marker;
        marker.header.frame_id = "base_link";
        marker.header.stamp = ros::Time::now();
        marker.ns = shape;
        marker.id = std::hash<std::string>()(id_str) % 10000;
        marker.type = visualization_msgs::Marker::SPHERE;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.position = position;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = 0.03;
        marker.scale.y = 0.03;
        marker.scale.z = 0.03;

        if (shape == "square") {
            marker.color.r = 1.0;
            marker.color.g = 0.0;
            marker.color.b = 0.0;
        } else {
            marker.color.r = 0.0;
            marker.color.g = 1.0;
            marker.color.b = 0.0;
        }
        marker.color.a = 1.0;

        marker.lifetime = ros::Duration();
        marker_pub.publish(marker);
    }
    std::vector<cv::Point> extractContour(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud) {
        // Convert the point cloud to an image
        float scale, offsetX, offsetY;
        cv::Mat img = pointCloudToImage(cloud, 500, 500, scale, offsetX, offsetY);
        
        // Apply Gaussian blur to smooth the image
        cv::GaussianBlur(img, img, cv::Size(5, 5), 0);
        
        // Perform edge detection using Canny
        cv::Mat edges;
        cv::Canny(img, edges, 50, 150);
    
        // Find contours in the edge-detected image
        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(edges, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    
        // If no contours found, return empty
        if (contours.empty()) return {};
    
        // Find the contour with the largest area
        double maxArea = 0;
        int maxIdx = 0;
        for (size_t i = 0; i < contours.size(); i++) {
            double area = cv::contourArea(contours[i]);
            if (area > maxArea) {
                maxArea = area;
                maxIdx = i;
            }
        }
    
        // Create a copy of the original image to draw contours
        cv::Mat img_copy = img.clone();
        
        // Draw the contours on the image
        cv::drawContours(img_copy, contours, maxIdx, cv::Scalar(0, 255, 0), 2);
    
        //Display the image with contours
        cv::imshow("Contour Visualization", img_copy);
        cv::waitKey(0);  // Wait for a key press to close the window
        
        // Return the contour with the largest area
        return contours[maxIdx];
    }
    
    bool solve(const cw2_world_spawner::Task3Service::Request &req,
               cw2_world_spawner::Task3Service::Response &res, cw2& robot, ros::NodeHandle &nh) {
        ROS_INFO("[Task3] Solving Task 3...");

        ros::Publisher marker_pub = nh.advertise<visualization_msgs::Marker>("visualization_marker", 10);

        double x_min = -0.6, x_max = 0.7;
        double y_min = -0.55, y_max = 0.55;
        double step_size = 0.3;

        geometry_msgs::Pose scan_pose;
        scan_pose.orientation.w = 1.0;
        scan_pose.position.z = 0.6;
        tf2::Quaternion quat;
        quat.setRPY(M_PI, 0, -M_PI / 4);
        scan_pose.orientation = tf2::toMsg(quat);

        for (double y = y_min; y <= y_max; y += step_size) {
            for (double x = x_min; x <= x_max; x += step_size) {
                scan_pose.position.x = x;
                scan_pose.position.y = y;

                if (robot.moveArm(scan_pose)) {
                    ros::Duration(1.0).sleep();

                    auto cloud = capturePointCloud(nh);
                    if (!cloud || cloud->empty()) continue;

                    auto contour = extractContour(cloud);
                    if (contour.empty()) continue;

                    

                    float scale, offsetX, offsetY;
                    cv::Mat img = pointCloudToImage(cloud, 500, 500, scale, offsetX, offsetY);
                    std::string shape = classifyShape(contour);
                    std::string size = estimateShapeSize(contour, scale);
                    cv::Point2f centroid = computeCentroid(contour);

                    pcl::PointXYZRGB referencePoint = cloud->points[0];
                    geometry_msgs::Point worldCentroid;
                    worldCentroid.x = referencePoint.x;
                    worldCentroid.y = referencePoint.y;
                    worldCentroid.z = referencePoint.z;

                    bool isNew = false;
                    if (shape == "square") {
                        isNew = isNewCentroid(worldCentroid, worldCentroidsSquare);
                        if (isNew) worldCentroidsSquare.push_back(worldCentroid);
                    } else if (shape == "cross") {
                        isNew = isNewCentroid(worldCentroid, worldCentroidsCross);
                        if (isNew) worldCentroidsCross.push_back(worldCentroid);
                    }

                    if (isNew) {
                        count[shape]++;
                        shapePointClouds.push_back(cloud);
                        publishMarker(worldCentroid, shape + std::to_string(count[shape]), shape, marker_pub);
                    }

                    ROS_INFO("Detected shape: %s (%s) at (%.2f, %.2f)", shape.c_str(), size.c_str(), centroid.x, centroid.y);
                }
            }
        }

        ROS_INFO("Final Counts - Squares: %d, Crosses: %d", count["square"], count["cross"]);
        return true;
    }
}