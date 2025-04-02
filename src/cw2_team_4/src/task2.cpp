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
#include <pcl/filters/voxel_grid.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/features/moment_of_inertia_estimation.h>
#include <pcl/common/pca.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/passthrough.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <set>
#include <iostream>

namespace task2 {

pcl::PointCloud<pcl::PointXYZ>::Ptr capturePointCloud(ros::NodeHandle &nh) {
    ROS_INFO("Waiting for a fresh point cloud...");
    
    std::vector<std::string> pointcloud_topics = {
        "/camera/depth_registered/points",
        "/r200/camera/depth_registered/points",
        "/r200/camera/depth/image_rect_raw",
        "/r200/camera/depth/image_raw"
    };

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
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

std::string classifyShape(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {
    std::cout << "Reached classifyShape" << std::endl;

    // Downsampling using Voxel Grid Filter
    pcl::VoxelGrid<pcl::PointXYZ> voxel;
    voxel.setInputCloud(cloud);
    voxel.setLeafSize(0.005f, 0.005f, 0.005f);
    voxel.filter(*cloud);

    pcl::PCA<pcl::PointXYZ> pca;
    pca.setInputCloud(cloud);

    Eigen::Vector3f eigenvalues = pca.getEigenValues();  // Sorted in descending order
    float eigenvalue1 = eigenvalues[0];  // Largest
    float eigenvalue2 = eigenvalues[1];  // Second largest
    
    std::cout << "\nPCA Eigenvalues:" << std::endl;
    std::cout << " - Largest Eigenvalue: " << eigenvalue1 << std::endl;
    std::cout << " - Second Largest Eigenvalue: " << eigenvalue2 << std::endl;

    // Compute the ratio of the two largest eigenvalues
    float eigen_ratio = eigenvalue1 / eigenvalue2;
    std::cout << " - Eigenvalue Ratio: " << eigen_ratio << std::endl;

    // Shape classification based on eigenvalue ratio
    std::string shape;
    if (eigen_ratio < 1.2) {
        shape = "Square";  // Shapes that are more cube-like
    } else {
        shape = "Cross";   // Elongated shapes
    }

    // Visualization part - Create a new viewer for each object
    pcl::visualization::PCLVisualizer viewer("Object Viewer");
    viewer.addPointCloud<pcl::PointXYZ>(cloud, "cloud");
    viewer.setBackgroundColor(0, 0, 0);  // Set background to black
    viewer.spin();  // Keep the window open until closed by the user

    std::cout << " - Classified Shape: " << shape << std::endl;
    
    return shape;
}

std::string takeObservation(geometry_msgs::PointStamped &observation_pose, ros::NodeHandle &nh) {
    std::cout << "Reached takeObservation" << std::endl;

    geometry_msgs::Pose curr_pose;
    curr_pose.position.x = observation_pose.point.x;
    curr_pose.position.y = observation_pose.point.y;
    curr_pose.position.z = observation_pose.point.z + 0.6;

    tf2::Quaternion quat;
    quat.setRPY(M_PI, 0, -M_PI / 4);
    curr_pose.orientation = tf2::toMsg(quat);

    cw2 robot(nh);
    if (!robot.moveArm(curr_pose)) return "Error";

    ros::Duration(1.0).sleep();  // Allow sensor time to refresh
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = capturePointCloud(nh);

    if (!cloud || cloud->empty()) {
        ROS_ERROR("Captured cloud is empty");
        return "Error";
    }

    std::string shape = classifyShape(cloud);
    cloud->clear();

    return shape;
}

bool solve(const cw2_world_spawner::Task2Service::Request &req,
           cw2_world_spawner::Task2Service::Response &res) {
    ROS_INFO("[Task2] Solving Task 2...");

    ros::NodeHandle nh;
    std::vector<geometry_msgs::PointStamped> object_points = req.ref_object_points;
    geometry_msgs::PointStamped goal_point = req.mystery_object_point;

    std::string shape1 = takeObservation(object_points.at(0), nh);
    std::string shape2 = takeObservation(object_points.at(1), nh);
    std::string unknownShape = takeObservation(goal_point, nh);

    ROS_INFO("Object 1: %s, Object 2: %s, Unknown: %s", shape1.c_str(), shape2.c_str(), unknownShape.c_str());

    if (shape1 == unknownShape) {
        ROS_INFO("Unknown object matches object 1");
    } else if (shape2 == unknownShape) {
        ROS_INFO("Unknown object matches object 2");
    } else {
        ROS_INFO("Unknown object doesn't match any of the two reference shapes.");
    }

    return true;
}

}
