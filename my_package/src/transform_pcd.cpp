#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>
#include <sensor_msgs/PointCloud2.h>
#include <my_package/transformPointsConfig.h>

#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/common/common.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl_ros/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/registration/ndt.h>

#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

typedef pcl::PointXYZI Point_t;
typedef pcl::PointCloud<Point_t> Cloud_t;

ros::Publisher map_pub;
Cloud_t src;
pcl::NormalDistributionsTransform<Point_t, Point_t> ndt;
bool has_map_cloud = false;

using namespace ros;
using namespace std;

void dynamic_configCb(my_package::transformPointsConfig &config, uint32_t level) 
{
  // Get transform values
  double x = config.x;
	double y = config.y;
	double z = config.z;
	double roll = config.roll;
	double pitch = config.pitch;
	double yaw = config.yaw;
  bool doNDT = config.doNDT;
  ROS_INFO("Reconfigure Requested. Proceeding to transform....");
  printf("Input pose: (%.4f, %.4f, %.4f, %.4f, %.4f, %.4f)\n", x, y, z, roll, pitch, yaw);

  Cloud_t dst;
  // Transformation matrix/Initial guess for NDT
  Eigen::Affine3f transform = pcl::getTransformation(x, y, z, roll, pitch, yaw);

  if(doNDT)
  {
    Cloud_t::Ptr src_ptr(new Cloud_t(src));
    Cloud_t::Ptr dst_ptr;
    pcl::VoxelGrid<Point_t> voxel_grid_filter;
    voxel_grid_filter.setLeafSize(2.5, 2.5, 2.5);
    voxel_grid_filter.setInputCloud(src_ptr);
    voxel_grid_filter.filter(*src_ptr);
    ndt.setInputSource(src_ptr);
    ndt.align(dst, transform.matrix());
    printf("NDT: %d iterations\n", ndt.getFinalNumIteration());
    tf::Matrix3x3 mat3x3;
    Eigen::Matrix4f ndt_tf = ndt.getFinalTransformation();
    x = ndt_tf(0, 3);
    y = ndt_tf(1, 3);
    z = ndt_tf(2, 3);
    mat3x3.setValue(static_cast<double>(ndt_tf(0, 0)), static_cast<double>(ndt_tf(0, 1)),
                    static_cast<double>(ndt_tf(0, 2)), static_cast<double>(ndt_tf(1, 0)),
                    static_cast<double>(ndt_tf(1, 1)), static_cast<double>(ndt_tf(1, 2)),
                    static_cast<double>(ndt_tf(2, 0)), static_cast<double>(ndt_tf(2, 1)),
                    static_cast<double>(ndt_tf(2, 2)));
    mat3x3.getRPY(roll, pitch, yaw);
    printf("NDT pose: (%.4f, %.4f, %.4f, %.4f, %.4f, %.4f)\n", x, y, z, roll, pitch, yaw);
    transform = pcl::getTransformation(x, y, z, roll, pitch, yaw);
  }

  // Transform pointcloud
  pcl::transformPointCloud(src, dst, transform);
	
  // Publish to topic to view in rviz
  Cloud_t::Ptr map_ptr(new Cloud_t(dst));
  sensor_msgs::PointCloud2::Ptr map_msg_ptr(new sensor_msgs::PointCloud2);
  pcl::toROSMsg(*map_ptr, *map_msg_ptr);
  map_msg_ptr->header.frame_id = "map";
  map_pub.publish(*map_msg_ptr);
  std::cout << "---------------------------------------\n" << std::endl;

  return;
}

void ndtMapCb(const sensor_msgs::PointCloud2::ConstPtr& input_msg)
{
  if(!has_map_cloud)
  {
    Cloud_t global_map;
    pcl::fromROSMsg(*input_msg, global_map);
    Cloud_t::Ptr global_map_ptr = Cloud_t::Ptr(new Cloud_t(global_map));
    ndt.setTransformationEpsilon(0.005);
		ndt.setStepSize(0.01);
		ndt.setResolution(1.2);
		ndt.setMaximumIterations(100);
		ndt.setInputTarget(global_map_ptr);
    has_map_cloud = true;
    std::cout << "Map loaded!" << std::endl;
  }
  return;
}

int main(int argc, char** argv)
{
  // Initiate node
  ros::init(argc, argv, "transform_pcd");

  ros::NodeHandle nh;
  ros::Subscriber map_sub = nh.subscribe("/points_map", 1, ndtMapCb);
  map_pub = nh.advertise<sensor_msgs::PointCloud2>("/ndt_map", 10, true);

  if(argc != 2)
  {
  	std::cout << "Please indicate pcd file" << std::endl;
  	std::cout << "Usage: rosrun my_package transform_pcd [file.pcd]" << std::endl;
  	return -1;
  }
  // Load pcd map to transform
  std::string pcd_filename = argv[1];
  std::cout << "Loading file: " << pcd_filename  << std::endl;
  if(pcl::io::loadPCDFile<Point_t>(pcd_filename, src) == -1)
  {
    std::cout << "Couldn't read " << pcd_filename << "." << std::endl;
    return -1;
  }
  std::cout << "Loaded " << src.size() << " data points from " << pcd_filename << std::endl;

  dynamic_reconfigure::Server<my_package::transformPointsConfig> server;
  dynamic_reconfigure::Server<my_package::transformPointsConfig>::CallbackType f;
  f = boost::bind(&dynamic_configCb, _1, _2);
  server.setCallback(f);

  ros::spin();
  return 0;
}
