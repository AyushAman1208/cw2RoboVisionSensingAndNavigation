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

    // // -------------------------
    // // Apply voxel grid filtering.
    // pcl::PointCloud<pcl::PointXYZRGB>::Ptr voxelFiltered(new pcl::PointCloud<pcl::PointXYZRGB>);
    // pcl::VoxelGrid<pcl::PointXYZRGB> voxel;
    // voxel.setInputCloud(cloud);
    // voxel.setLeafSize(0.001f, 0.001f, 0.001f);  // Adjust leaf size (5 mm) as needed.
    // voxel.filter(*voxelFiltered);

    // ROS_INFO("After voxel filtering, %lu points remain.", voxelFiltered->points.size());


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

  // Extract clusters from a point cloud and filter clusters based on size.
// Only clusters with maximum dimension (x or y) between 0.08 m and 0.22 m are returned.
std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> extractClusters(
  const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud)
{
  // Create a KdTree for clustering.
  pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>);
  tree->setInputCloud(cloud);

  // Perform Euclidean Cluster Extraction.
  std::vector<pcl::PointIndices> clusterIndices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec;
  ec.setClusterTolerance(0.02f); // 2 cm tolerance (adjust based on your data density)
  ec.setMinClusterSize(50);      // minimum number of points in a cluster
  ec.setMaxClusterSize(25000);   // maximum number of points in a cluster
  ec.setSearchMethod(tree);
  ec.setInputCloud(cloud);
  ec.extract(clusterIndices);

  std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> clusters;
  for (const auto &indices : clusterIndices)
  {
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr cluster(new pcl::PointCloud<pcl::PointXYZRGB>);
      for (const auto &idx : indices.indices)
      {
          cluster->points.push_back(cloud->points[idx]);
      }
      cluster->width = cluster->points.size();
      cluster->height = 1;
      cluster->is_dense = true;

      // Compute the bounding box (x-y only) for the cluster.
      float minX = std::numeric_limits<float>::max();
      float maxX = -std::numeric_limits<float>::max();
      float minY = std::numeric_limits<float>::max();
      float maxY = -std::numeric_limits<float>::max();
      for (const auto &pt : cluster->points)
      {
          if (pt.x < minX) minX = pt.x;
          if (pt.x > maxX) maxX = pt.x;
          if (pt.y < minY) minY = pt.y;
          if (pt.y > maxY) maxY = pt.y;
      }
      float clusterWidth  = maxX - minX;
      float clusterHeight = maxY - minY;
      float clusterSize   = std::max(clusterWidth, clusterHeight);

      // Filter clusters to those within the tolerance: 80 mm to 220 mm.
      if (clusterSize >= 0.08f && clusterSize <= 0.22f)
      {
          clusters.push_back(cluster);
      }
  }
  return clusters;
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

    // Apply Canny edge detection on the adaptive-thresholded image.
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
      
      // // Smooth the contour to remove small irregularities
      // std::vector<cv::Point> smoothedContour;
      // cv::approxPolyDP(filteredContours[maxIdx], smoothedContour, 
      //                 0.003 * cv::arcLength(filteredContours[maxIdx], true), true);
      
      // // If the approximation is too aggressive, use original contour
      // if (smoothedContour.size() < 5) {
      //     smoothedContour = filteredContours[maxIdx];
      // }
      
      // // Create visualization for debugging
      // cv::Mat debug;
      // cv::cvtColor(img, debug, cv::COLOR_GRAY2BGR);
      
      // // Draw all filtered contours in blue
      // cv::drawContours(debug, filteredContours, -1, cv::Scalar(255, 0, 0), 1);
      
      // // Draw the largest contour in green
      // cv::drawContours(debug, std::vector<std::vector<cv::Point>>{smoothedContour}, 0, cv::Scalar(0, 255, 0), 2);
      
      // // Show intermediate processing steps
      // // cv::imshow("Original Image", img);
      // // cv::imshow("Preprocessed Binary", morphed);
      // cv::imshow("Contour Detection", debug);
      // cv::waitKey(0);  // Use non-zero value for debugging
      
      // return smoothedContour;

std::string classifyShape(const std::vector<cv::Point>& contour)
{    // Skip if contour is too small
  if (contour.size() < 5) {
    return "none";
}

// Calculate basic properties
cv::Rect boundingRect = cv::boundingRect(contour);
double contourArea = cv::contourArea(contour);

// Filter out very small contours that might be noise
if (contourArea < 100) { // Adjust this threshold based on your image scale
  return "none";
}

double boundingRectArea = boundingRect.width * boundingRect.height;
    // Find moments for center of mass
    cv::Moments moments = cv::moments(contour);
    if (moments.m00 <= 0) {
        return "none";
    }
    
    double centerX = moments.m10 / moments.m00;
    double centerY = moments.m01 / moments.m00;
    cv::Point centerPoint(static_cast<int>(centerX), static_cast<int>(centerY));
    
    // Create a mask from the contour
    cv::Mat mask = cv::Mat::zeros(boundingRect.height + 2, boundingRect.width + 2, CV_8UC1);
    std::vector<cv::Point> shiftedContour;
    
    for (const auto& point : contour) {
        shiftedContour.push_back(cv::Point(point.x - boundingRect.x + 1, point.y - boundingRect.y + 1));
    }
    
    std::vector<std::vector<cv::Point>> shiftedContours = {shiftedContour};
    cv::drawContours(mask, shiftedContours, 0, cv::Scalar(255), cv::FILLED);
    
    // Find contours in the mask to detect holes
    std::vector<std::vector<cv::Point>> innerContours;
    cv::Mat maskCopy = mask.clone();
    cv::findContours(maskCopy, innerContours, cv::RETR_CCOMP, cv::CHAIN_APPROX_SIMPLE);
    
    // Analyze holes
    bool hasHole = false;
    double holeArea = 0;
    
    if (innerContours.size() > 1) {
        // Sort contours by area (descending)
        std::sort(innerContours.begin(), innerContours.end(), 
                 [](const std::vector<cv::Point>& c1, const std::vector<cv::Point>& c2) {
                     return cv::contourArea(c1) > cv::contourArea(c2);
                 });
        
        // Check if the second largest contour is actually a hole
        if (innerContours.size() > 1) {
            holeArea = cv::contourArea(innerContours[1]);
            // A significant hole (relative to contour area) indicates a nought
            hasHole = (holeArea > 0.1 * contourArea);
        }
    }
    
    // Calculate solidity (ratio of contour area to convex hull area)
    std::vector<cv::Point> hull;
    cv::convexHull(contour, hull);
    double hullArea = cv::contourArea(hull);
    double solidity = (hullArea > 0) ? contourArea / hullArea : 0;
    
    // Calculate extent (ratio of contour area to bounding rectangle area)
    double extent = contourArea / boundingRectArea;
    
    // Calculate aspect ratio
    double aspectRatio = static_cast<double>(boundingRect.width) / boundingRect.height;
    
    // For cross detection: analyze the shape through orientation analysis
    // A cross should have four main "arms" extending from the center
    
    // Create distance profile from center to edges
    const int numAngles = 36; // Check every 10 degrees
    std::vector<double> radialDistances(numAngles, 0);
    std::vector<bool> isEdgePoint(numAngles, false);
    
    for (int i = 0; i < numAngles; i++) {
        double angle = i * (2 * CV_PI / numAngles);
        double maxDist = 0;
        
        // Scan from center to edge in this direction
        for (double r = 0; r < std::max(boundingRect.width, boundingRect.height); r += 1.0) {
            int x = centerX + r * cos(angle);
            int y = centerY + r * sin(angle);
            
            // Check if point is inside the mask boundaries
            if (x - boundingRect.x + 1 >= 0 && x - boundingRect.x + 1 < mask.cols &&
                y - boundingRect.y + 1 >= 0 && y - boundingRect.y + 1 < mask.rows) {
                
                // Check if this point is still inside the shape
                if (mask.at<uchar>(y - boundingRect.y + 1, x - boundingRect.x + 1) > 0) {
                    maxDist = r;
                } else {
                    // We've hit the edge of the shape
                    break;
                }
            } else {
                // We've gone outside the mask boundaries
                break;
            }
        }
        
        radialDistances[i] = maxDist;
        if (maxDist > 0) {
            isEdgePoint[i] = true;
        }
    }
    
    // Analyze the radial distance profile for peaks (arms of the cross)
    int peakCount = 0;
    bool inPeak = false;
    double peakThreshold = 0.5 * *std::max_element(radialDistances.begin(), radialDistances.end());
    
    for (int i = 0; i < numAngles; i++) {
        if (!inPeak && radialDistances[i] > peakThreshold) {
            inPeak = true;
            peakCount++;
        } else if (inPeak && radialDistances[i] <= peakThreshold) {
            inPeak = false;
        }
    }
    
    // Check the last and first elements for continuity
    if (inPeak && radialDistances[0] <= peakThreshold) {
        // Close the last peak if it wraps around
        inPeak = false;
    } else if (!inPeak && radialDistances[0] > peakThreshold && radialDistances[numAngles-1] > peakThreshold) {
        // Adjust peak count if the peak wraps around
        peakCount--;
    }
    
    // Calculate standard deviation of non-zero distances to analyze distance uniformity
    std::vector<double> nonZeroDistances;
    for (const auto& dist : radialDistances) {
        if (dist > 0) {
            nonZeroDistances.push_back(dist);
        }
    }
    
    double meanDist = std::accumulate(nonZeroDistances.begin(), nonZeroDistances.end(), 0.0) / nonZeroDistances.size();
    double sumVar = 0.0;
    for (const auto& dist : nonZeroDistances) {
        sumVar += (dist - meanDist) * (dist - meanDist);
    }
    double stdDev = std::sqrt(sumVar / nonZeroDistances.size());
    double coeffVar = meanDist > 0 ? stdDev / meanDist : 0;
    
    // For a NOUGHT:
    // - Should have a significant hole
    // - High solidity (close to 1)
    // - Aspect ratio close to 1 for square shape
    // - Low coefficient of variation in radial distances (uniform distance from center)
    
    // For a CROSS:
    // - No significant hole
    // - Should have around 4 peaks in radial distance profile (4 arms)
    // - Higher coefficient of variation due to arms
    
    // Improved classification logic
    if (hasHole && holeArea > 0.2 * contourArea && 
        solidity > 0.5 && 
        aspectRatio > 0.6 && aspectRatio < 1.4 && 
        coeffVar < 0.5) {
        return "nought";
    }
    else if (!hasHole && 
             peakCount >= 3 && peakCount <= 5 && // Allow some tolerance for imperfect crosses
             solidity < 0.8 && 
             coeffVar > 0.2) {
        return "cross";
    }
    
    return "none";
}

std::string classifyShapeFromPointCloud(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud) {
  if (cloud->empty()) {
      return "none";
  }

  // Step 1: Find the centroid of the point cloud
  Eigen::Vector4f centroid;
  pcl::compute3DCentroid(*cloud, centroid);
  
  // Step 2: Find the 2D bounding box (assuming points are generally on a plane)
  float minX = std::numeric_limits<float>::max();
  float minY = std::numeric_limits<float>::max();
  float maxX = -std::numeric_limits<float>::max();
  float maxY = -std::numeric_limits<float>::max();
  
  for (const auto& point : cloud->points) {
      minX = std::min(minX, point.x);
      minY = std::min(minY, point.y);
      maxX = std::max(maxX, point.x);
      maxY = std::max(maxY, point.y);
  }
  
  float width = maxX - minX;
  float height = maxY - minY;
  float aspectRatio = width / height;
  
  // Step 3: Create a 2D grid to represent the point distribution
  const int gridSize = 20; // Adjust based on point cloud density
  std::vector<std::vector<bool>> occupancyGrid(gridSize, std::vector<bool>(gridSize, false));
  
  // Fill the grid based on point presence
  for (const auto& point : cloud->points) {
      int gridX = static_cast<int>((point.x - minX) / width * (gridSize - 1));
      int gridY = static_cast<int>((point.y - minY) / height * (gridSize - 1));
      
      // Bound checking
      gridX = std::max(0, std::min(gridSize - 1, gridX));
      gridY = std::max(0, std::min(gridSize - 1, gridY));
      
      occupancyGrid[gridY][gridX] = true;
  }
  
  // Step 4: Analyze the grid for shape characteristics
  
  // Count occupied cells
  int occupiedCells = 0;
  for (const auto& row : occupancyGrid) {
      for (bool cell : row) {
          if (cell) occupiedCells++;
      }
  }
  
  // Calculate density (percentage of grid filled)
  float density = static_cast<float>(occupiedCells) / (gridSize * gridSize);
  
  // Step 5: Check for empty center (for nought detection)
  // Get center region of the grid
  int centerStartX = gridSize / 4;
  int centerEndX = (gridSize * 3) / 4;
  int centerStartY = gridSize / 4;
  int centerEndY = (gridSize * 3) / 4;
  
  int centerCells = 0;
  int centerOccupied = 0;
  
  for (int y = centerStartY; y < centerEndY; y++) {
      for (int x = centerStartX; x < centerEndX; x++) {
          centerCells++;
          if (occupancyGrid[y][x]) centerOccupied++;
      }
  }
  
  float centerDensity = static_cast<float>(centerOccupied) / centerCells;
  
  // Step 6: Check for cross pattern using radial density analysis
  
     // Define regions (in grid coordinates)
     struct Region {
      int startX, startY, endX, endY;
  };
  
  // Define the four "arms" regions of a potential cross
  Region leftArm = {0, gridSize/3, gridSize/3, (2*gridSize)/3};
  Region rightArm = {(2*gridSize)/3, gridSize/3, gridSize, (2*gridSize)/3};
  Region topArm = {gridSize/3, 0, (2*gridSize)/3, gridSize/3};
  Region bottomArm = {gridSize/3, (2*gridSize)/3, (2*gridSize)/3, gridSize};
  
  // Define corners
  Region topLeft = {0, 0, gridSize/3, gridSize/3};
  Region topRight = {(2*gridSize)/3, 0, gridSize, gridSize/3};
  Region bottomLeft = {0, (2*gridSize)/3, gridSize/3, gridSize};
  Region bottomRight = {(2*gridSize)/3, (2*gridSize)/3, gridSize, gridSize};
  
  auto calculateRegionDensity = [&occupancyGrid](const Region& r) -> float {
      int cells = 0;
      int occupied = 0;
      for (int y = r.startY; y < r.endY; y++) {
          for (int x = r.startX; x < r.endX; x++) {
              cells++;
              if (occupancyGrid[y][x]) occupied++;
          }
      }
      return cells > 0 ? static_cast<float>(occupied) / cells : 0;
  };
  
  float leftDensity = calculateRegionDensity(leftArm);
  float rightDensity = calculateRegionDensity(rightArm);
  float topDensity = calculateRegionDensity(topArm);
  float bottomDensity = calculateRegionDensity(bottomArm);
  
  float topLeftDensity = calculateRegionDensity(topLeft);
  float topRightDensity = calculateRegionDensity(topRight);
  float bottomLeftDensity = calculateRegionDensity(bottomLeft);
  float bottomRightDensity = calculateRegionDensity(bottomRight);
  
  // Average arm density and corner density
  float armsDensity = (leftDensity + rightDensity + topDensity + bottomDensity) / 4.0;
  float cornersDensity = (topLeftDensity + topRightDensity + bottomLeftDensity + bottomRightDensity) / 4.0;
  
  // Step 7: Calculate radial density (rings around center)
  std::vector<float> ringDensities;
  const int numRings = 5;
  
  for (int ring = 0; ring < numRings; ring++) {
      float innerRadiusRatio = static_cast<float>(ring) / numRings;
      float outerRadiusRatio = static_cast<float>(ring + 1) / numRings;
      
      int pointsInRing = 0;
      int totalPointsChecked = 0;
      
      for (const auto& point : cloud->points) {
          // Calculate normalized distance from centroid (0-1 range)
          float dx = (point.x - centroid[0]) / (width/2);
          float dy = (point.y - centroid[1]) / (height/2);
          float normalizedDist = std::sqrt(dx*dx + dy*dy);
          
          if (normalizedDist >= innerRadiusRatio && normalizedDist < outerRadiusRatio) {
              totalPointsChecked++;
              pointsInRing++;
          }
      }
      
      // Avoid division by zero
      float ringDensity = totalPointsChecked > 0 ? 
          static_cast<float>(pointsInRing) / totalPointsChecked : 0.0f;
      
      ringDensities.push_back(ringDensity);
  }
  
  // Debug output
  ROS_INFO("Shape metrics:");
  ROS_INFO("  Total points: %lu", cloud->points.size());
  ROS_INFO("  Overall density: %.2f", density);
  ROS_INFO("  Center density: %.2f", centerDensity);
  ROS_INFO("  Arms density: %.2f", armsDensity);
  ROS_INFO("  Corners density: %.2f", cornersDensity);
  ROS_INFO("  Aspect ratio: %.2f", aspectRatio);
  ROS_INFO("  Ring densities: [%.2f, %.2f, %.2f, %.2f, %.2f]", 
           ringDensities[0], ringDensities[1], ringDensities[2], 
           ringDensities[3], ringDensities[4]);
  
  // OPTIMIZED CLASSIFICATION LOGIC BASED ON THE PROVIDED METRICS
  
  // ------ NOUGHT DETECTION -------
  // From your metrics, a nought has:
  // - Near-zero center density (0.00)
  // - Moderate overall density (0.61)
  // - High corner density (0.79)
  // - Ring pattern with empty inner rings and filled outer rings [0.00, 0.00, 1.00, 1.00, 1.00]
  // - Square aspect ratio (0.99)
  
  bool isNought = centerDensity < 0.20 &&                 // Empty center
                  density > 0.40 &&                       // Substantial overall density
                  cornersDensity > 0.40 &&                // Corners are filled
                  aspectRatio > 0.8 && aspectRatio < 1.2; // Square-ish shape
  
  // Additional check for ring pattern typical for noughts
  if (ringDensities.size() >= 3) {
      // First two rings should be near empty, outer rings filled
      if (ringDensities[0] < 0.30 && ringDensities[1] < 0.30 && 
          ringDensities[3] > 0.70 && ringDensities[4] > 0.70) {
          isNought = isNought && true;
      } else {
          isNought = false;
      }
  }
  
  // ------ CROSS DETECTION -------
  // From your metrics, a cross has:
  // - High center density (0.75)
  // - Moderate overall density (0.44)
  // - Zero corner density (0.00)
  // - High arm density (0.71)
  // - Uniform ring pattern [1.00, 1.00, 1.00, 1.00, 1.00]
  // - Square aspect ratio (1.00)
  
  bool isCross = centerDensity > 0.50 &&                  // Filled center
                 cornersDensity < 0.30 &&                 // Empty corners
                 armsDensity > 0.50 &&                    // Substantial arm density
                 aspectRatio > 0.8 && aspectRatio < 1.2;  // Square-ish shape
  
  // Check for uniform ring density (characteristic of crosses)
  if (ringDensities.size() >= 3) {
      float ringSum = 0;
      float ringVar = 0;
      
      // Calculate mean
      for (float density : ringDensities) {
          ringSum += density;
      }
      float ringMean = ringSum / ringDensities.size();
      
      // Calculate variance
      for (float density : ringDensities) {
          ringVar += (density - ringMean) * (density - ringMean);
      }
      ringVar /= ringDensities.size();
      
      // Low variance indicates uniform density across rings (cross)
      // High variance indicates non-uniform density (nought)
      if (ringVar < 0.05 && ringMean > 0.70) {
          isCross = isCross && true;
      }
  }
  
  // Make the final classification
  if (isNought) {
      return "nought";
  } 
  else if (isCross) {
      return "cross";
  }
  
  return "none";
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
  pose.position.z = 0.85;

  // std::vector<double> x_values = {-0.45, -0.25, 0, 0.25, 0.55, 0.55, 0.55, 0.55, 0.25, 0.35, 0.35, -0.35, -0.5, -0.5, -0.35, -0.45, -0.25, 0};
  // std::vector<double> y_values = {-0.4, -0.4, -0.4, -0.4, -0.4, -0.2, 0.2, 0.4, 0.4, 0.2, -0.2, -0.2, -0.2, 0.2, 0.2, 0.4, 0.4, 0.4};

  std::vector<double> x_values = {0.4, 0, -0.4, 0};
  std::vector<double> y_values = {0, 0.4, 0, -0.4};


  for (size_t i = 0; i < x_values.size(); ++i) {
          pose.position.x = x_values[i];
          pose.position.y = y_values[i];
          scan_poses.push_back(pose);
  }


// Iterate over these 18 scan poses.
for (const auto &scan_pose : scan_poses) {

     if (robot.moveArm(scan_pose)) {
         ros::Duration(1.0).sleep();

         auto cloud = capturePointCloud(nh);
         if (!cloud || cloud->empty()) continue;

         auto clusters = extractClusters(cloud);
         if (clusters.empty()) continue;
          
         // For each cluster, extract the largest contour.

         for (const auto &cluster : clusters) {
          // Process each cluster as you would a full cloud.
          // auto contour = extractContour(cluster);
          // if (contour.empty()) continue;
      
          // float scale, offsetX, offsetY;
          // cv::Mat img = pointCloudToImage(cluster, 500, 500, scale, offsetX, offsetY);
          // std::string shape = classifyShape(contour);
          std::string shape = classifyShapeFromPointCloud(cluster);
          if (shape == "none") continue;
          // Estimate size and centroid.
          auto sizePair = estimateSize(cluster);
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
}


ROS_INFO("Final Counts - Noughts: %d, Crosses: %d", count["nought"], count["cross"]);
return true;
}

} // namespace task3