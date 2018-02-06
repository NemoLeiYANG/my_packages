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
// #include "boost/filesystem.hpp"

// #include <iostream>
#include <fstream>
#include <sstream>
#include <string>

int main(int argc, char** argv)
{
  if(argc != 4)
  {
    std::cout << "ERROR: Wrong argument number." << std::endl;
    std::cout << "Usage: rosrun my_package import_to_bag TXT_DIR TIMESTAMP.csv OUTPUT.bag" << std::endl;
    return(-1);
  }

  std::string txt_dir = argv[1];
  if(txt_dir.back() != '/')
    txt_dir.push_back('/');
  std::string csv_filename = argv[2];
  std::string bag_filename = argv[3];
  // Heads-up reminder for usage
  std::cout << "INFO: Reading data in the " << txt_dir << " directory." << std::endl;
  std::cout << "Please ensure that ONLY .txt files with the correct format are in " << txt_dir << " directory." << std::endl;
  std::cout << "INFO: Reading " << csv_filename << " file in current directory." << std::endl;

  // Output bag file
  rosbag::Bag bag;
  std::cout << "Output: " << bag_filename << std::endl;
  bag.open(bag_filename, rosbag::bagmode::Write);

  // Timestamp data
  std::ifstream time_stamp_stream;
  try
  {
    time_stamp_stream.open(csv_filename);
  }
  catch(std::exception& e)
  {
    std::cout << e.what() << std::endl;
    std::cout << "ERROR: It seems that " << csv_filename << " is not available? Instead, use: \n";
    std::cout << "\trosrun my_package import_to_bag_nostamp\n";
    return(-1);
  }
  std::string frame_id = "velodyne";

  // count
  unsigned int file_count = 0;
  std::string time_stamp_str;
  while(getline(time_stamp_stream, time_stamp_str))
  {
    file_count++;
  }

  // Processing
  time_stamp_stream.close();
  time_stamp_stream.open(csv_filename);
  unsigned int file_number = 1;
  unsigned int seq = 0;
  uint64_t sec, nsec;
  while(getline(time_stamp_stream, time_stamp_str)) try
  {
    std::ifstream in_stream;
    std::cout << "--------------------------------------------------------------\n";
    std::cout << "Reading: " << txt_dir << file_number << ".txt" << std::endl;
    in_stream.open(txt_dir + std::to_string(file_number) + ".txt");

    // Place-holder for variables
    pcl::PointCloud<pcl::PointXYZI> scan;
    std::string line, x_str, y_str, z_str, intensity_str;
    int points_skipped = 0;
    while(getline(in_stream, line)) // Loop through lines in file
    {
      std::stringstream line_stream(line); // convert string to separatable stream
      
      // Get date
      getline(line_stream, x_str, ' ');
      getline(line_stream, y_str, ' ');
      getline(line_stream, z_str, ' ');
      getline(line_stream, intensity_str);
      double x = std::stod(x_str);
      double y = std::stod(y_str);
      double z = std::stod(z_str);
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

    if(scan.size() == 0)
    {
      std::cout << "INFO: No points to read in " << txt_dir << file_number << ".txt" << std::endl;
      file_number++;
      continue;
    }
    // Write to pcd file
    // pcl::io::savePCDFileBinary("pcds/Lidar" + std::to_string(file_number) + ".pcd", scan);
    std::cout << "(Func disabled) Saved [" << scan.size() << " points, " << points_skipped 
              << " skipped] points to pcds/Lidar" << file_number << ".pcd" << std::endl;

    // Timestamp for msg
    // Time in microseconds i assume
    sec = std::stoull(time_stamp_str.c_str()) / 1000000; // to s
    nsec = std::stoull(time_stamp_str.c_str()) % 1000000 * 1000; // to us to ns
    std::cout << "sec: " << sec << "\n";
    std::cout << "nsec: " << nsec << std::endl;

    // Write to bag file
    sensor_msgs::PointCloud2::Ptr scan_msg_ptr(new sensor_msgs::PointCloud2);
    pcl::toROSMsg(scan, *scan_msg_ptr);
    scan_msg_ptr->header.seq = seq;
    scan_msg_ptr->header.stamp.sec = sec;
    scan_msg_ptr->header.stamp.nsec = nsec;
    scan_msg_ptr->header.frame_id = frame_id;
    bag.write("/points_raw", ros::Time(sec, nsec), *scan_msg_ptr); 
    // bag.write("/points_raw", ros::Time::now(), *scan_msg_ptr); 
    // pub.publish(*scan_msg_ptr);
    file_number++;
    seq++;  
  }
  catch(std::exception& e)
  {
    std::cout << e.what() << std::endl;
    std::cout << "No more data for timestamp? Skipping the rest of " << csv_filename << std::endl;
  }

  bag.close();
  std::cout << "Processed: " << file_number - 1 << " / Expected: " << file_count << std::endl;
  std::cout << "Finished. Wrote bag file to " << bag_filename << std::endl;

  return 0;
}
