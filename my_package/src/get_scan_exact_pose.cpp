// #include <ros/ros.h>
// #include <sensor_msgs/PointCloud2.h>

#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <iostream>
#include <fstream>
#include <string>
#include <vector>

// #define OFFSET_TO_BASE

struct pose
{
  double x;
  double y;
  double z;
  double roll;
  double pitch;
  double yaw;
  int key;
};

inline double calculateMinAngleDist(double first, double second) // in radian
{
  double difference = first - second;
  if(difference >= 3.14159265359)
    return difference - 6.28318530718;
  if(difference <= -3.14159265359)
    return difference + 6.28318530718;
  return difference;
}

int main(int argc, char** argv)
{
  // Load input
  std::string cloudfile = "/home/zwu/LIDAR-DATA/7mar-tube.pcd"; // pointcloud map
  std::string posefile = "/home/zwu/data-0321/tube/stairs-1/pose.csv"; // localizing_pose
  std::string file_location = "/home/zwu/data-0321/tube/stairs-1/lidar_scan/"; // output dir
  unsigned int start_index = 1;

  pcl::PointCloud<pcl::PointXYZI> src;
  if(pcl::io::loadPCDFile<pcl::PointXYZI>(cloudfile, src) == -1)
  {
    std::cout << "Couldn't read " << cloudfile << "." << std::endl;
    return(-1);
  }
  std::cout << "Loaded " << src.size() << " data points from " << cloudfile << std::endl;
  
  // Downsample the source cloud, apply voxelgrid filter
  // double voxel_leaf_size = 0.3;
  // pcl::PointCloud<pcl::PointXYZI>::Ptr src_ptr(new pcl::PointCloud<pcl::PointXYZI>(src));
  // pcl::VoxelGrid<pcl::PointXYZI> voxel_grid_filter;
  // voxel_grid_filter.setLeafSize(voxel_leaf_size, voxel_leaf_size, voxel_leaf_size);
  // voxel_grid_filter.setInputCloud(src_ptr);
  // voxel_grid_filter.filter(src);

  // And set up KDTree
  pcl::PointCloud<pcl::PointXYZ> tmp_xyz;
  for(pcl::PointCloud<pcl::PointXYZI>::const_iterator item = src.begin(); item != src.end(); item++)
  {
    pcl::PointXYZ p;
    p.x = (double)item->x;
    p.y = (double)item->y;
    p.z = (double)item->z;
    tmp_xyz.push_back(p);
  }
  pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
  pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_ptr(new pcl::PointCloud<pcl::PointXYZ>(tmp_xyz));
  kdtree.setInputCloud(filtered_ptr);

  // LIDAR pose data
  std::ifstream pose_stream;
  pose_stream.open(posefile);

  // Place-holder for csv stream variables
  std::string line, key_str, seq_str, sec_str, nsec_str, x_str, y_str, z_str, roll_str, pitch_str, yaw_str;
  std::vector<pose> lidar_poses;
  // getline(pose_stream, line); // skip header line
  // std::cout << "File sequence: " << line << std::endl;
  std::cout << "Expected sequence: x,y,z,roll,pitch,yaw" << std::endl;
  std::cout << "Collecting localized lidar poses and time stamps." << std::endl;
  
  #ifdef OFFSET_TO_BASE
  Eigen::Affine3d offset_tf;
  pcl::getTransformation(1.2, 0, 2.0, 0, 0, 0, offset_tf);
  #endif
  
  while(getline(pose_stream, line))
  {
    std::stringstream line_stream(line);

    // Get data value
    getline(line_stream, x_str, ',');
    getline(line_stream, y_str, ',');
    getline(line_stream, z_str, ',');
    getline(line_stream, roll_str, ',');
    getline(line_stream, pitch_str, ',');
    getline(line_stream, yaw_str);

    Eigen::Affine3d tf_vtow;
    pcl::getTransformation(std::stod(x_str), std::stod(y_str), std::stod(z_str),
                           std::stod(roll_str), std::stod(pitch_str), std::stod(yaw_str),
                           tf_vtow);

    pose current_pose({std::stod(x_str), std::stod(y_str), std::stod(z_str),
                       std::stod(roll_str), std::stod(pitch_str), std::stod(yaw_str)});

    lidar_poses.push_back(current_pose);
  }
  pose_stream.close();
  std::cout << "INFO: Collected " << lidar_poses.size() << " data." << std::endl;

  /////////////////////////////////////////////////////////////////////////////////////////////////////////
  // Do interpolation and publish the synthetic perspective pointcloud
  unsigned int file_count = start_index;
  for(unsigned int lIdx = 0, lIdx_end = lidar_poses.size(); lIdx < lIdx_end; lIdx++)
  {
    // Search around interpolated point for cloud
    // K nearest neighbor search
    pcl::PointXYZ searchPoint;
    searchPoint.x = lidar_poses[lIdx].x;
    searchPoint.y = lidar_poses[lIdx].y;
    searchPoint.z = lidar_poses[lIdx].z;

    // Neighbors within radius search
    std::vector<int> pointIdxRadiusSearch;
    std::vector<float> pointRadiusSquaredDistance;

    float radius = 50.0;
    pcl::PointCloud<pcl::PointXYZI> nearestCloud;
    if(kdtree.radiusSearch(searchPoint, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0)
    {
      for(size_t i = 0; i < pointIdxRadiusSearch.size(); ++i)
      {
        nearestCloud.push_back(src.points[pointIdxRadiusSearch[i]]);
      }
    }

    // Transform back to local coordinate
    Eigen::Affine3d transform;
    pcl::getTransformation(lidar_poses[lIdx].x, 
                           lidar_poses[lIdx].y, 
                           lidar_poses[lIdx].z, 
                           lidar_poses[lIdx].roll, 
                           lidar_poses[lIdx].pitch, 
                           lidar_poses[lIdx].yaw, transform);

    #ifdef OFFSET_TO_BASE
    transform = transform * offset_tf;
    #endif

    pcl::PointCloud<pcl::PointXYZI>::Ptr localCloud(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::transformPointCloud(nearestCloud, *localCloud, transform.inverse());

    // Convert pointcloud to txt file and save
    std::ofstream pointcloud_stream;
    pointcloud_stream.open(file_location + std::to_string(file_count) + ".txt");
    for(pcl::PointCloud<pcl::PointXYZI>::const_iterator item = localCloud->begin(); item != localCloud->end(); item++)
    {
      char output_buffer[1000];
      double x = item->x;
      double y = item->y;
      double z = item->z;
      int intensity = item->intensity;
      sprintf(output_buffer, "%.10lf %.10lf %.10lf %d", x, y, z, intensity);
      pointcloud_stream << output_buffer << std::endl;
    }
    std::cout << "Saved " << localCloud->size() << " points to " + file_location + std::to_string(file_count) + ".txt" << std::endl;
    std::cout << "---------------------------------------------------------" << std::endl;
    file_count++;
  }
  // ros::spin();
  return 0;
}