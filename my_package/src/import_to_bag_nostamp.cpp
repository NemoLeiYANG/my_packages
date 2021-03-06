/***
Author: TA
This code takes in a number of text files in a folder, read each as a pointcloud
and convert to PointCloud2 msg and dump all into a bag file
Please go to the data directory that contain the files and run the node without any argument
instead of putting the directory as an argument because I have not implement that yet.
***/

#include <ros/ros.h>
#include <rosbag/bag.h>
#include <ros/time.h>
#include <sensor_msgs/PointCloud2.h>
// #include <tf/transform_datatypes.h>

// #include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/common.h>
// #include <pcl_ros/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
// #include <Eigen/Geometry>
#include "boost/filesystem.hpp"

// #include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <boost/filesystem.hpp>

using namespace std;
int main(int argc, char** argv)
{
  // Heads-up reminder for usage
  if(argc < 3)
  {
    cout << "Missing input arguments!" << endl;
    cout << "Usage: rosrun my_package import_to_bag_nostamp TXT_DIR OUTPUT.bag" << endl;
  }
  std::cout << "Ensure that ONLY .txt files with the correct format are in txts/ directory." << std::endl;
  // Input dir
  std::string input_dir = argv[1];
  if(input_dir.back() != '/')
    input_dir.push_back('/');
  int file_count = std::distance(boost::filesystem::directory_iterator(input_dir), {});

  // Output bag
  std::string outbag_name = argv[2];
  rosbag::Bag bag;
  bag.open(outbag_name, rosbag::bagmode::Write);

  // Some fake data for pointcloud
  int seq = 1;
  int32_t sec = 1504851231; // this should be around 8 sep, noon
  int32_t nsec = 000000001;
  std::string frame_id = "velodyne";

  // Get all file in the specified directory
  for(int file_number = 1; file_number <= file_count; file_number++)
  {
    std::string filename = std::to_string(file_number) + ".txt";

    std::ifstream in_stream;  
    in_stream.open(input_dir + "/" + filename);
    cout << "Reading: " << filename << flush;

    // Place-holder for variables
    pcl::PointCloud<pcl::PointXYZI> scan; 
    std::string line, x_str, y_str, z_str, intensity_str;
    int points_skipped = 0;
    while(getline(in_stream, line)) // Loop through lines in file
    {
      std::stringstream line_stream(line); // convert string to separatable stream
      
      // Get date
      getline(line_stream, x_str, ' ');
      double x = std::stod(x_str);
      getline(line_stream, y_str, ' ');
      double y = std::stod(y_str);
      getline(line_stream, z_str, ' ');
      double z = std::stod(z_str);
      getline(line_stream, intensity_str);
      double intensity = std::stod(intensity_str);

      if(!x && !y && !z && !intensity)
      {
        points_skipped++;
        continue;
      }
      // Convert data to pointXYZI and push it to cloud [scan]
      // pcl::PointXYZI new_point(x, y, z, intensity);
      pcl::PointXYZI new_point;
      new_point.x = x;
      new_point.y = y;
      new_point.z = z;
      new_point.intensity = intensity;

      scan.push_back(new_point);
    }
    // Write to pcd file
    cout << "... saved [" << scan.size() << " points, " << points_skipped << " skipped] " << endl;
    std::cout << "---------------------------------------" << std::endl;

    // Write to bag file
    sensor_msgs::PointCloud2::Ptr scan_msg_ptr(new sensor_msgs::PointCloud2);
    pcl::toROSMsg(scan, *scan_msg_ptr);
    scan_msg_ptr->header.seq = seq;
    scan_msg_ptr->header.stamp.sec = sec;
    scan_msg_ptr->header.stamp.nsec = nsec;
    scan_msg_ptr->header.frame_id = frame_id;
    bag.write("/points_raw", ros::Time(sec, nsec), *scan_msg_ptr);

    // Update fake data before processing next scan
    seq++;
    nsec += 100000000; // rate is 10Hz, thus 0.1s = 10E8 ns
    if(nsec >= 1000000000)
    {
      sec++;
      nsec -= 1000000000;
    }
  }

  bag.close();
  std::cout << "Finished. Outputed bag file to " << outbag_name << std::endl;

  return 0;
}