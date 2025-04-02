#include "task2.h"
#include "cw2_class.h"
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Pose.h>
#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/filters/passthrough.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <set>
#include <iostream>

namespace task2 {

pcl::PointCloud<pcl::PointXYZRGB>::Ptr capturePointCloud(ros::NodeHandle &nh) {
    ROS_INFO("Waiting for a fresh point cloud...");
    
    std::vector<std::string> pointcloud_topics = {
        "/camera/depth_registered/points",
        "/r200/camera/depth_registered/points",
        "/r200/camera/depth/image_rect_raw",
        "/r200/camera/depth/image_raw"
    };

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    bool received = false;

    for (const auto& topic : pointcloud_topics) {
        ROS_INFO("Trying topic: %s", topic.c_str());

        ros::topic::waitForMessage<sensor_msgs::PointCloud2>(topic, nh, ros::Duration(0.5));
        boost::shared_ptr<const sensor_msgs::PointCloud2> msg =
            ros::topic::waitForMessage<sensor_msgs::PointCloud2>(topic, nh, ros::Duration(5.0));

        if (msg) {
            pcl::fromROSMsg(*msg, *cloud);
            ROS_INFO("Point cloud received successfully from %s", topic.c_str());
            received = true;
            break;
        } else {
            ROS_WARN("No point cloud received from %s", topic.c_str());
        }
    }

    if (!received) {
        ROS_ERROR("Failed to receive point cloud from all available sources.");
    return nullptr;
    }

    return cloud;
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr cropPointCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, geometry_msgs::PointStamped &observation_pose) {
    pcl::PassThrough<pcl::PointXYZRGB> pass;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cropped_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    
    float range = 0.1;
    pass.setInputCloud(cloud);
    pass.setFilterFieldName("x");
    pass.setFilterLimits(observation_pose.point.x - range, observation_pose.point.x + range);
    pass.filter(*cropped_cloud);
    
    pass.setInputCloud(cropped_cloud);
    pass.setFilterFieldName("y");
    pass.setFilterLimits(observation_pose.point.y - range, observation_pose.point.y + range);
    pass.filter(*cropped_cloud);
    
    pass.setInputCloud(cropped_cloud);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(observation_pose.point.z - range, observation_pose.point.z + range);
    pass.filter(*cropped_cloud);

    // Visualization part - Create a new viewer for each object
    pcl::visualization::PCLVisualizer viewer("Object Viewer");
    viewer.addPointCloud<pcl::PointXYZRGB>(cropped_cloud, "cloud");
    viewer.setBackgroundColor(0, 0, 0);  // Set background to black
    viewer.spin();  // Keep the window open until closed by the user
    
    return cropped_cloud;
}

bool comparePointClouds(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud1, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud2) {
    if (cloud1->size() != cloud2->size()) return false;
    Eigen::Vector4f centroid1, centroid2;
    pcl::compute3DCentroid(*cloud1, centroid1);
    pcl::compute3DCentroid(*cloud2, centroid2);
    return (centroid1 - centroid2).norm() < 0.02;
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr takeObservation(geometry_msgs::PointStamped &observation_pose, ros::NodeHandle &nh) {
    cw2 robot(nh);
    geometry_msgs::Pose curr_pose;
    curr_pose.position.x = observation_pose.point.x;
    curr_pose.position.y = observation_pose.point.y;
    curr_pose.position.z = observation_pose.point.z + 0.6;
    tf2::Quaternion quat;
    quat.setRPY(M_PI, 0, -M_PI / 4);
    curr_pose.orientation = tf2::toMsg(quat);
    
    if (!robot.moveArm(curr_pose)) return nullptr;
    ros::Duration(1.0).sleep();
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud = capturePointCloud(nh);
    if (!cloud || cloud->empty()) return nullptr;

    // Visualization part - Create a new viewer for each object
    pcl::visualization::PCLVisualizer viewer("Object Viewer");
    viewer.addPointCloud<pcl::PointXYZRGB>(cloud, "cloud");
    viewer.setBackgroundColor(0, 0, 0);  // Set background to black
    viewer.spin();  // Keep the window open until closed by the user
    return cloud;
}

bool solve(const cw2_world_spawner::Task2Service::Request &req, cw2_world_spawner::Task2Service::Response &res) {
    ROS_INFO("[Task2] Solving Task 2...");
    ros::NodeHandle nh;
    std::vector<geometry_msgs::PointStamped> object_points = req.ref_object_points;
    geometry_msgs::PointStamped goal_point = req.mystery_object_point;
    auto ref_cloud1 = takeObservation(object_points.at(0), nh);
    auto ref_cloud2 = takeObservation(object_points.at(1), nh);
    auto goal_cloud = takeObservation(goal_point, nh);
    

    if (!ref_cloud1 || !ref_cloud2 || !goal_cloud) {
        ROS_ERROR("One or more point clouds are empty");
        return false;
    }
    if (comparePointClouds(ref_cloud1, goal_cloud)) {
        ROS_INFO("Unknown object matches object 1");
    } else if (comparePointClouds(ref_cloud2, goal_cloud)) {
        ROS_INFO("Unknown object matches object 2");
    } else {
        ROS_INFO("Unknown object does not match any reference object");
    }
    return true;
}
}
