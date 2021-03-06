#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <cstddef>

#include <ros/ros.h>
#include <ros/time.h>
#include <ros/duration.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/PointCloud2.h>

#include <boost/foreach.hpp> // to read bag file
#define foreach BOOST_FOREACH

#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include <pcl/common/common.h>
#include <pcl_ros/transforms.h>
#include <pcl_conversions/pcl_conversions.h>

#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>

struct pose
{
  double x;
  double y;
  double z;
  double roll;
  double pitch;
  double yaw;
};

struct velocity
{
  double x;
  double y;
  double z;
  double roll;
  double pitch;
  double yaw;
};

inline double getYawAngle(double _x, double _y)
{
  return std::atan2(_y, _x) * 180 / 3.14159265359; // degree value
}

inline double calculateMinAngleDist(double first, double second) // in degree
{
  double difference = first - second;
  if(difference >= 180.0)
    return difference - 360.0;
  if(difference <= -180.0)
    return difference + 360.0;
  return difference;
}

void correctLIDARscan(pcl::PointCloud<pcl::PointXYZI>& scan, Eigen::Affine3d relative_tf, double scan_interval)
{
  // Correct scan using vel
  pcl::PointCloud<pcl::PointXYZI> scan_packet;
  std::vector<pcl::PointCloud<pcl::PointXYZI>> scan_packets_vector;
  double base_azimuth = getYawAngle((scan.begin())->x, (scan.begin())->y);
  for(pcl::PointCloud<pcl::PointXYZI>::const_iterator item = scan.begin(); item != scan.end(); item++)
  {
    double crnt_azimuth = getYawAngle(item->x, item->y);
    if(std::fabs(calculateMinAngleDist(crnt_azimuth, base_azimuth)) < 0.01) // 0.17 degree is the typical change
    {
      scan_packet.push_back(*item);
    }
    else // new azimuth reached
    {
      scan_packets_vector.push_back(scan_packet);
      scan_packet.clear();
      scan_packet.push_back(*item);
      base_azimuth = crnt_azimuth;
    }
  }

  scan.clear();
  pose crnt_pose = {0, 0, 0, 0, 0, 0};
  velocity vel;
  pcl::getTranslationAndEulerAngles(relative_tf, vel.x, vel.y, vel.z, vel.roll, vel.pitch, vel.yaw);
  vel.x = vel.x / scan_interval;
  vel.y = vel.y / scan_interval;
  vel.z = vel.z / scan_interval;
  vel.roll = vel.roll / scan_interval;
  vel.pitch = vel.pitch / scan_interval;
  vel.yaw = vel.yaw / scan_interval;
  for(int i = 0, npackets = scan_packets_vector.size(); i < npackets; i++)
  {
    double offset_time = scan_interval * i / npackets;
    pose this_packet_pose = {crnt_pose.x - vel.x * offset_time,
                             crnt_pose.y - vel.y * offset_time,
                             crnt_pose.z - vel.z * offset_time,
                             crnt_pose.roll - vel.roll * offset_time,
                             crnt_pose.pitch - vel.pitch * offset_time,
                             crnt_pose.yaw - vel.yaw * offset_time}; 

    Eigen::Affine3d transform;
    pcl::getTransformation(this_packet_pose.x, 
                           this_packet_pose.y, 
                           this_packet_pose.z, 
                           this_packet_pose.roll, 
                           this_packet_pose.pitch, 
                           this_packet_pose.yaw, transform);
    pcl::PointCloud<pcl::PointXYZI> corrected_packet;
    pcl::transformPointCloud(scan_packets_vector[npackets-1-i], corrected_packet, transform);
    scan += corrected_packet;
  }
}

int main(int argc, char** argv)
{
  // Initiate and get csv file
  ros::init(argc, argv, "submap_merger");
  ros::NodeHandle nh;
  ros::Rate rosrate(5);
  ros::Publisher scan_pub = nh.advertise<sensor_msgs::PointCloud2>("/current_scan", 100, true);
  ros::Publisher map_pub = nh.advertise<sensor_msgs::PointCloud2>("/ndt_map", 100, true);

  if(argc < 4)
  {
    // std::cout << "Please indicate the bag file corresponding to the map_pose.csv." << std::endl;
    std::cout << "ERROR: Missing user inputs." << std::endl;
    std::cout << "!! rosrun my_package submap_merger \"posefile.csv\" \"mapfile.pcd\" \"bagfile.bag\" [min_scan_range]" << std::endl;
    return -1;
  }

  // pose data
  std::string csvfile = argv[1];
  std::cout << "Reading " << csvfile << " in the current directory.\n";
  std::cout << "Please ensure that the format of the csv file is:" << std::endl;
  std::cout << "\t{sequence, sec, nsec, x, y, z, roll, pitch, yaw}" << std::endl;
  std::ifstream csv_stream(csvfile);

  // map data
  pcl::PointCloud<pcl::PointXYZI> map_src;
  if(pcl::io::loadPCDFile<pcl::PointXYZI>(argv[2], map_src) == -1)
  {
    std::cout << "Couldn't read " << argv[2] << "." << std::endl;
    return(-1);
  }
  std::cout << "Loaded " << map_src.size() << " data points from " << argv[2] << std::endl;
  sensor_msgs::PointCloud2::Ptr map_msg_ptr(new sensor_msgs::PointCloud2);
  pcl::toROSMsg(map_src, *map_msg_ptr);
  map_msg_ptr->header.frame_id = "map";
  map_pub.publish(map_msg_ptr);

  // scan data
  std::string bagfile = argv[3];
  std::string bagtopic = "/points_raw";
  std::cout << "Reading bag: " << bagfile << " [topic: " << bagtopic << "]... " << std::flush;
  rosbag::Bag bag(bagfile, rosbag::bagmode::Read);
  std::vector<std::string> reading_topics;
    reading_topics.push_back(std::string(bagtopic));
  rosbag::View view(bag, rosbag::TopicQuery(reading_topics));
  std::cout << "Done." << std::endl;

  double min_scan_range = 0;
  if(argc == 5)
  {
    min_scan_range = std::stod(argv[4]);
    std::cout << "Using min_scan_range = " << min_scan_range << std::endl;
  }

  // Place-holder for variables
  pcl::PointCloud<pcl::PointXYZI> map;
  std::string line, key_str, seq_str, sec_str, nsec_str, x_str, y_str, z_str, roll_str, pitch_str, yaw_str;
  unsigned int seq, key;
  double x, y, z, roll, pitch, yaw;
  Eigen::Affine3d prev_transform;
  ros::Time prev_time;
  bool isFirstScan = true;
  bool isGetLine = false;

  getline(csv_stream, line); // to skip header line
  std::cout << "Finished. Starting merging submaps..." << std::endl;
  foreach(rosbag::MessageInstance const message, view)
  {
    // get new data and process values
    if(!isGetLine)
    {  
      if(!getline(csv_stream, line))
      {
        break; // end of csv
      }
      isGetLine = true;
    }

    std::cout << line << std::endl;
    std::stringstream line_stream(line);

    // getline(line_stream, key_str, ',');
    // key = std::stod(key_str);

    getline(line_stream, seq_str, ',');
    seq = std::stod(seq_str);
    getline(line_stream, sec_str, ',');
    getline(line_stream, nsec_str, ',');
    ros::Time crnt_time(std::stod(sec_str), std::stod(nsec_str));
    getline(line_stream, x_str, ',');
    x = std::stod(x_str);
    getline(line_stream, y_str, ',');
    y = std::stod(y_str);
    getline(line_stream, z_str, ',');
    z = std::stod(z_str);
    getline(line_stream, roll_str, ',');
    roll = std::stod(roll_str);
    getline(line_stream, pitch_str, ',');
    pitch = std::stod(pitch_str);
    getline(line_stream, yaw_str);
    yaw = std::stod(yaw_str);

    sensor_msgs::PointCloud2::ConstPtr input_cloud = message.instantiate<sensor_msgs::PointCloud2>();
    if(input_cloud == NULL) // no data?
    {
      std::cout << "No input PointCloud available. Waiting..." << std::endl;
      continue;
    }
    else // check whether the sequence match
    {
      if(input_cloud->header.seq != seq)
      {
        std::cout << "Info: input_cloud->header.seq != seq (" << input_cloud->header.seq << " != " << seq << ")" << std::endl;
        continue;
      }

      if(input_cloud->header.stamp != crnt_time)
      {
        std::cout << "Error: input_cloud->header.stamp != crnt_time" << std::endl;
        std::cout << "(" << input_cloud->header.stamp << " != " << crnt_time << ")" << std::endl;
        return(-1);
      }
    }

    // Create transformation matrix
    Eigen::Affine3d crnt_transform, rel_transform;
    pcl::getTransformation(x, y, z, roll, pitch, yaw, crnt_transform);
    double interval;
    if(isFirstScan)
    {
      prev_transform = crnt_transform;
      interval = 0.1; // dummy value, just need to be != 0
      rel_transform = prev_transform.inverse() * crnt_transform;
      isFirstScan = false;
    }
    else
    {
      interval = (crnt_time - prev_time).toSec();
      rel_transform = prev_transform.inverse() * crnt_transform;
    }

    // Correct point cloud first
    pcl::PointCloud<pcl::PointXYZI> src, fsrc, dst;
    pcl::fromROSMsg(*input_cloud, src);
    correctLIDARscan(src, rel_transform, interval);

    if(argc == 5)
    {
      // Filter pointcloud
      pcl::PointXYZI p;
      for(pcl::PointCloud<pcl::PointXYZI>::const_iterator item = src.begin(); item != src.end(); item++)
      {
        p.x = (double)item->x;
        p.y = (double)item->y;
        p.z = (double)item->z;
        p.intensity = (double)item->intensity;
        if((sqrt(pow(p.x, 2.0) + pow(p.y, 2.0))) > min_scan_range)
        {
          fsrc.push_back(p);
        }
      }
    }
    else fsrc = src;
    // pcl::io::savePCDFileBinary(key_str + ".pcd", fsrc);

    // Transform the corrected pointcloud
    pcl::transformPointCloud(fsrc, dst, crnt_transform);

    // Add to map
    map += dst;

    tf::TransformBroadcaster br;
    tf::Transform transform;
    tf::Quaternion q;
    transform.setOrigin(tf::Vector3(x, y, z));
    q.setRPY(roll, pitch, yaw);
    transform.setRotation(q);
    br.sendTransform(tf::StampedTransform(transform, input_cloud->header.stamp, "map", "base_link"));

    sensor_msgs::PointCloud2::Ptr scan_msg_ptr(new sensor_msgs::PointCloud2);
    pcl::toROSMsg(dst, *scan_msg_ptr);
    scan_msg_ptr->header.frame_id = "map";
    scan_pub.publish(scan_msg_ptr);

    // Show output
    std::cout << "Number: " << key << "\n";
    std::cout << "Sequence: " << seq << "\n";
    std::cout << "Number of scan points: " << dst.size() << "\n";
    std::cout << "Number of map points: " << map.size() << "\n";
    std::cout << "(" << x << ", " << y << ", " << z << ", " << roll << ", " << pitch << ", " << yaw << ")\n";
    // std::cout << "Transformation Matrix: \n";
    // std::cout << crnt_transform.matrix() << std::endl;
    std::cout << "---------------------------------------" << std::endl;

    // Update prev values
    prev_transform = crnt_transform;
    prev_time = crnt_time;
    isGetLine = false;
    rosrate.sleep();
  }
  bag.close();
  std::cout << "Finished processing bag file." << std::endl;
  return 0;
}