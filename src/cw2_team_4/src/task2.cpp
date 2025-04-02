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

    pcl::PointCloud<pcl::PointXYZ>::Ptr capturePointCloud() {
        ROS_INFO("Waiting for point cloud...");
    
        // Use waitForMessage with a timeout (e.g., 5 seconds)
        boost::shared_ptr<const sensor_msgs::PointCloud2> msg =
            ros::topic::waitForMessage<sensor_msgs::PointCloud2>("/camera/depth_registered/points", ros::Duration(5.0));
    
        if (!msg) {
            ROS_WARN("No point cloud received from /camera/depth_registered/points, trying alternative topic...");
    
            // Attempt to get the point cloud from an alternative topic
            msg = ros::topic::waitForMessage<sensor_msgs::PointCloud2>("/r200/camera/depth_registered/points", ros::Duration(5.0));
    
            if (!msg) {
                ROS_ERROR("No point cloud received from both topics within timeout.");
                return nullptr;
            }
        }
    
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(*msg, *cloud);
    
        ROS_INFO("Point cloud received successfully.");
        return cloud;
    }
    void extractZCoordinatesAndFilterObject(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {
    std::set<float> unique_z_values; // Set to store unique z-coordinates

    // Iterate over all points in the cloud and extract unique z-coordinates
    for (const auto& point : cloud->points) {
        unique_z_values.insert(point.z); // Insert Z value into the set
    }

    // Print unique z-coordinates
    std::cout << "Unique Z-coordinates: " << std::endl;
    for (const auto& z : unique_z_values) {
        std::cout << z << std::endl;
    }

    // Define z-value range for filtering (modify based on your object's z-range)
    float z_min = 0.49f; // Minimum z-value for the object
    float z_max = 0.55f; // Maximum z-value for the object

    // Apply Pass-Through filter based on z-coordinate
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud(cloud);
    pass.setFilterFieldName("z");  // Filter based on the z-coordinate
    pass.setFilterLimits(z_min, z_max);  // Filter points within the specified range
    pass.filter(*cloud);  // Apply filter

    // Print the number of points after filtering
    std::cout << "\nNumber of points after filtering: " << cloud->size() << std::endl;

    // Optionally, print the filtered points
    std::cout << "Filtered Points (first 5 shown):" << std::endl;
    for (size_t i = 0; i < std::min<size_t>(cloud->size(), 5); ++i) {
        std::cout << "Point " << i << ": (" 
                  << cloud->points[i].x << ", " 
                  << cloud->points[i].y << ", " 
                  << cloud->points[i].z << ")" << std::endl;
    }

    // Visualization part
    pcl::visualization::PCLVisualizer viewer("Filtered Point Cloud");
    viewer.addPointCloud<pcl::PointXYZ>(cloud, "filtered_cloud");

    // Set up background color (optional)
    viewer.setBackgroundColor(0, 0, 0);  // Black background

    // Start the visualization loop
    viewer.spin();  // Keep the window open until closed by the user
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


    std::string takeObservation(geometry_msgs::PointStamped &observation_pose) {
        std::cout << "Reached takeObservation";
        std::cout << std::endl;
        geometry_msgs::Pose curr_pose;
        curr_pose.position.x = observation_pose.point.x;
        curr_pose.position.y = observation_pose.point.y;
        curr_pose.position.z = observation_pose.point.z + 0.6;
    
        tf2::Quaternion quat;
        quat.setRPY(M_PI, 0, -M_PI / 4);
        curr_pose.orientation = tf2::toMsg(quat);
    
        ros::NodeHandle nh;
        cw2 robot(nh);
        if (!robot.moveArm(curr_pose)) return "Error";
    
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = capturePointCloud();
        
        // Clear any previous data in cloud before processing
        
        std::string shape = classifyShape(cloud);
        cloud->clear();
        
        return shape;
    }
    

bool solve(const cw2_world_spawner::Task2Service::Request &req,
           cw2_world_spawner::Task2Service::Response &res) {
    ROS_INFO("[Task2] Solving Task 2...");

    std::vector<geometry_msgs::PointStamped> object_points = req.ref_object_points;
    geometry_msgs::PointStamped goal_point = req.mystery_object_point;

    std::string shape1 = takeObservation(object_points.at(0));
    std::string shape2 = takeObservation(object_points.at(1));
    std::string unknownShape = takeObservation(goal_point);

    ROS_INFO("Object 1: %s, Object 2: %s, Unknown: %s", shape1.c_str(), shape2.c_str(), unknownShape.c_str());

    std::cout<<"Unknown shape:  "<<unknownShape;
    std::cout<<std::endl;
    if(shape1 == unknownShape){
        ROS_INFO("Unknown object matches object 1");
    }
    else if(shape2 == unknownShape){
        ROS_INFO("Unknown object matches object 2");
    }
    else{
        ROS_INFO("Unknown object dosen't match any of the two reference shapes.");
    }
    return true;
}

}
