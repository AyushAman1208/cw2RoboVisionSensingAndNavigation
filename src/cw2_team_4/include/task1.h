#ifndef TASK1_H
#define TASK1_H

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <opencv2/opencv.hpp>
#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <tf2_ros/buffer.h>
#include <string>
#include <vector>
#include <tuple>

// If you have a cw2_class header, include it here as needed.
#include "cw2_class.h"

// Use your service message header if required, for example:
#include "cw2_world_spawner/Task1Service.h"

namespace task1 {

// Capture a point cloud from the camera (with color filtering).
pcl::PointCloud<pcl::PointXYZRGB>::Ptr capturePointCloud(ros::NodeHandle &nh);

// Convert a point cloud to a grayscale image using dynamic scaling.
cv::Mat pointCloudToImage(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud,
                          int imageWidth, int imageHeight,
                          float &scale, float &offsetX, float &offsetY);

// Generate an ideal contour for a given shape ("cross" or "nought").
std::vector<cv::Point> getIdealContour(const std::string &shapeType,
                                        float scale, float offsetX, float offsetY);

// Compute the orientation (in radians) of an object based on its point cloud and shape type.
float computeOrientationCV(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud,
                           const std::string &shapeType);

// Determine the pick offset given a shape type, object size (in mm), and rotation angle.
std::vector<float> determinePickOffset(const std::string &shapeType, int size, float rotation_angle);

// Transform a point (in camera frame) to the base frame.
std::vector<float> transformPointCameraToBase(const std::vector<float>& point_camera_frame,
                                              tf2_ros::Buffer &tf_buffer);

// Extract clusters from a point cloud.
std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> extractClusters(
    const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud);

// Check if a given centroid is new (i.e. not too close to an existing centroid).
bool isNewCentroid(const geometry_msgs::Point &worldCentroid,
                   const std::vector<geometry_msgs::Point>& existingCentroids,
                   double threshold = 0.05);

// A function that classifies and measures a shape from a cluster.
// Returns a tuple: (shape, estimatedSize, centroid, rotation_angle).
std::tuple<std::string, int, std::vector<float>, float> classifyAndMeasureShape(
    const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cluster);

// Alternatively, if you have a simpler classifier that works directly on the cloud:
std::string classifyShapeFromPointCloud(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cluster);

// The main solve function for Task1.
bool solve(const cw2_world_spawner::Task1Service::Request &req,
           cw2_world_spawner::Task1Service::Response &res,
           cw2 &robot, ros::NodeHandle &nh);

} // namespace task1

#endif // TASK1_H
