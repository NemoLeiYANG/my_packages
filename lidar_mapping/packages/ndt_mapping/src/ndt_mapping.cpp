// Basic libs
#include <chrono>
#include <fstream>
#include <iostream>
#include <mutex>
#include <omp.h>
#include <sstream>
#include <string>
#include <unordered_map>
#include <vector>

// Libraries for system commands
#include <sys/types.h>
#include <sys/stat.h>

#include <boost/foreach.hpp> // to read bag file
#define foreach BOOST_FOREACH

// ROS libs
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <ros/time.h>
#include <ros/duration.h>
#include <signal.h>
#include <sensor_msgs/PointCloud2.h>

#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>

// PCL & 3rd party libs
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#ifdef USE_FAST_PCL
  #include <fast_pcl/registration/ndt.h>
  #include <fast_pcl/filters/voxel_grid.h>
#else
  #include <pcl/registration/ndt.h>
  #include <pcl/filters/voxel_grid.h>
#endif

#ifdef USE_GPU_PCL
#include <fast_pcl/ndt_gpu/NormalDistributionsTransform.h>
#endif

#include <lidar_pcl/motion_undistortion.h>

// Here are the functions I wrote. De-comment to use
#define TILE_WIDTH 35 // Maximum range of LIDAR 32E is 70m
#define MY_EXTRACT_SCANPOSE // do not use this, this is to extract scans and poses to close loop
// #define LIMIT_HEIGHT 3.0 // filter out high points when aligning
// #define CORRECT_SCAN_DEBUG
// #define DOWNSAMPLE_ADD_MAP

#ifdef MY_EXTRACT_SCANPOSE
std::ofstream csv_stream;
std::string csv_filename = "map_pose.csv";
#endif // MY_EXTRACT_SCANPOSE

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

// struct Key
// {
//   int x;
//   int y;
// };

// bool operator==(const Key& lhs, const Key& rhs)
// {
//   return lhs.x == rhs.x && lhs.y == rhs.y;
// }

// bool operator!=(const Key& lhs, const Key& rhs)
// {
//   return lhs.x != rhs.x || lhs.y != rhs.y;
// }

// namespace std
// {
//   template <>
//   struct hash<Key> // custom hashing function for Key<(x,y)>
//   {
//     std::size_t operator()(const Key& k) const
//     {
//       return hash<int>()(k.x) ^ (hash<int>()(k.y) << 1);
//     }
//   };
// }

// global variables
static pose previous_pose, guess_pose, current_pose, ndt_pose, added_pose, localizer_pose;
static Eigen::Affine3d current_pose_tf, previous_pose_tf, relative_pose_tf;
static ros::Publisher ndt_map_pub, current_scan_pub, original_scan_pub;

static ros::Time current_scan_time;
static ros::Time previous_scan_time;
static ros::Duration scan_duration;

static double diff, diff_x, diff_y, diff_z, diff_roll, diff_pitch, diff_yaw; // current_pose - previous_pose
static double secs = 0.100085; // scan duration
// static velocity current_velocity;

static std::unordered_map<Key, pcl::PointCloud<pcl::PointXYZI>> world_map;
static pcl::PointCloud<pcl::PointXYZI> local_map;
static std::mutex mtx;
static Key local_key, previous_key;

#ifdef USE_GPU_PCL
static gpu::GNormalDistributionsTransform gpu_ndt;
#else
static pcl::NormalDistributionsTransform<pcl::PointXYZI, pcl::PointXYZI> ndt;
#endif

// Default NDT algorithm param values
static int max_iter = 300;       // Maximum iterations
static float ndt_res = 2.8;      // Resolution
static double step_size = 0.05;   // Step size
static double trans_eps = 0.001;  // Transformation epsilon

static double voxel_leaf_size = 0.1;
static double min_scan_range = 2.0;
static double min_add_scan_shift = 1.0;
static double min_add_scan_yaw_diff = 0.005;

// Workspace params
static float _start_time = 0; // 0 means start playing bag from beginnning
static float _play_duration = -1; // negative means play everything
static std::string _bag_file;
static std::string _output_directory;
static std::string _namespace;

static double _tf_x, _tf_y, _tf_z, _tf_roll, _tf_pitch, _tf_yaw;
static Eigen::Matrix4f tf_btol, tf_ltob;

static int add_scan_number = 1; // added frame count
static int initial_scan_loaded = 0;
static bool isMapUpdate = true;
static double fitness_score;
static bool has_converged;
static int final_num_iteration;

// File name get from time
std::time_t process_begin = std::time(NULL);
std::tm* pnow = std::localtime(&process_begin);

static inline double getYawAngle(double _x, double _y)
{
  return std::atan2(_y, _x) * 180 / 3.14159265359; // degree value
}

static inline double calculateMinAngleDist(double first, double second) // in degree
{
  double difference = first - second;
  if(difference >= 180.0)
    return difference - 360.0;
  if(difference <= -180.0)
    return difference + 360.0;
  return difference;
}

static void correctLIDARscan(pcl::PointCloud<pcl::PointXYZI>& scan, Eigen::Affine3d relative_tf, double scan_interval)
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

static void add_new_scan(const pcl::PointCloud<pcl::PointXYZI> new_scan)
{
 #ifdef DOWNSAMPLE_ADD_MAP
  pcl::PointCloud<pcl::PointXYZI>::Ptr new_scan_ptr(new pcl::PointCloud<pcl::PointXYZI>(new_scan));
  pcl::VoxelGrid<pcl::PointXYZI> voxel_grid_filter;
  voxel_grid_filter.setLeafSize(0.2, 0.2, 0.2);
  voxel_grid_filter.setInputCloud(new_scan_ptr);
  voxel_grid_filter.filter(*new_scan_ptr);
  for(pcl::PointCloud<pcl::PointXYZI>::const_iterator item = new_scan_ptr->begin(); item < new_scan_ptr->end(); item++)
 #else
  for(pcl::PointCloud<pcl::PointXYZI>::const_iterator item = new_scan.begin(); item < new_scan.end(); item++)
 #endif // DOWNSAMPLE_ADD_MAP
  {
    // Get 2D point
    Key key;
    key.x = int(floor(item->x / TILE_WIDTH));
    key.y = int(floor(item->y / TILE_WIDTH));

    world_map[key].push_back(*item);
  }
 #ifdef DOWNSAMPLE_ADD_MAP
  local_map += *new_scan_ptr;
 #else
  local_map += new_scan;
 #endif // DOWNSAMPLE_ADD_MAP
}

static void ndt_mapping_callback(const sensor_msgs::PointCloud2::ConstPtr& input)
{
  double r;
  pcl::PointXYZI p;
  pcl::PointCloud<pcl::PointXYZI> tmp, scan;
  pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_scan_ptr(new pcl::PointCloud<pcl::PointXYZI>());
  pcl::PointCloud<pcl::PointXYZI>::Ptr transformed_scan_ptr(new pcl::PointCloud<pcl::PointXYZI>());
  tf::Quaternion q;

  Eigen::Matrix4f t_localizer(Eigen::Matrix4f::Identity());
  Eigen::Matrix4f t_base_link(Eigen::Matrix4f::Identity());
  tf::TransformBroadcaster br;
  tf::Transform transform;

  current_scan_time = input->header.stamp;

  pcl::fromROSMsg(*input, tmp);
  // lidar_pcl::fromROSMsg(*input, tmp); // note here

  for(pcl::PointCloud<pcl::PointXYZI>::const_iterator item = tmp.begin(); item != tmp.end(); item++)
  {
    p.x = (double)item->x;
    p.y = (double)item->y;
    p.z = (double)item->z;
    p.intensity = (double)item->intensity;

    r = sqrt(pow(p.x, 2.0) + pow(p.y, 2.0));
    if(r > min_scan_range)
    {
      scan.push_back(p);
    }
  }

  // correctLIDARscan(scan, relative_pose_tf, secs);
  lidar_pcl::motionUndistort(scan, relative_pose_tf);
  pcl::PointCloud<pcl::PointXYZI>::Ptr scan_ptr(new pcl::PointCloud<pcl::PointXYZI>(scan));

  #ifdef LIMIT_HEIGHT
  pcl::PointCloud<pcl::PointXYZI> src;
  if(/*input->header.seq > 1780 &&*/ input->header.seq < 2300)
    for(pcl::PointCloud<pcl::PointXYZI>::const_iterator item = scan.begin(); item != scan.end(); item++)
    {
      // Eigen::Vector3d p(item->x, item->y, item->z);
      // Eigen::Vector3d f = relative_pose_tf.inverse() * p;
      double point_azi = getYawAngle(item->x, item->y);
      // std::cout << point
      if(item->z < LIMIT_HEIGHT 
         && ((point_azi < -20 && point_azi > -180) || (point_azi > 170)))
         // && (point_azi > -20 && point_azi < 170))
      {
        src.push_back(*item);
      }
    }
  else 
    src = scan;
  pcl::PointCloud<pcl::PointXYZI>::Ptr src_ptr(new pcl::PointCloud<pcl::PointXYZI>(src));
  #endif // LIMIT_HEIGHT

  // Add initial point cloud to velodyne_map
  if(initial_scan_loaded == 0)
  {
    pcl::transformPointCloud(*scan_ptr, *transformed_scan_ptr, tf_btol);
    add_new_scan(*transformed_scan_ptr);
    initial_scan_loaded = 1;
#ifdef MY_EXTRACT_SCANPOSE
    // outputing into csv
    csv_stream << add_scan_number << "," << input->header.seq << "," << current_scan_time.sec << "," << current_scan_time.nsec << ","
               << _tf_x << "," << _tf_y << "," << _tf_z << "," 
               << _tf_roll << "," << _tf_pitch << "," << _tf_yaw
               << std::endl;
#endif // MY_EXTRACT_SCANPOSE
    add_scan_number++;
    return;
  }
  // Apply voxelgrid filter
  pcl::VoxelGrid<pcl::PointXYZI> voxel_grid_filter;
  voxel_grid_filter.setLeafSize(voxel_leaf_size, voxel_leaf_size, voxel_leaf_size);
  #ifdef LIMIT_HEIGHT
  voxel_grid_filter.setInputCloud(src_ptr);
  #else
  voxel_grid_filter.setInputCloud(scan_ptr);
  #endif // LIMIT_HEIGHT
  voxel_grid_filter.filter(*filtered_scan_ptr);

  pcl::PointCloud<pcl::PointXYZI>::Ptr local_map_ptr(new pcl::PointCloud<pcl::PointXYZI>(local_map));

#ifdef USE_GPU_PCL
  gpu_ndt.setInputSource(filtered_scan_ptr);
#else
  ndt.setInputSource(filtered_scan_ptr);
#endif

  std::chrono::time_point<std::chrono::system_clock> t1 = std::chrono::system_clock::now();
  if(isMapUpdate == true)
  {
  #ifdef USE_GPU_PCL
    gpu_ndt.setInputTarget(local_map_ptr);
  #else
    ndt.setInputTarget(local_map_ptr);
  #endif
    isMapUpdate = false;
  }
  std::chrono::time_point<std::chrono::system_clock> t2 = std::chrono::system_clock::now();
  double ndt_update_time = std::chrono::duration_cast<std::chrono::microseconds>(t2 - t1).count() / 1000.0;

  guess_pose.x = previous_pose.x + diff_x;
  guess_pose.y = previous_pose.y + diff_y;
  guess_pose.z = previous_pose.z + diff_z;
  guess_pose.roll = previous_pose.roll;
  guess_pose.pitch = previous_pose.pitch;
  guess_pose.yaw = previous_pose.yaw + diff_yaw;

  Eigen::AngleAxisf init_rotation_x(guess_pose.roll, Eigen::Vector3f::UnitX());
  Eigen::AngleAxisf init_rotation_y(guess_pose.pitch, Eigen::Vector3f::UnitY());
  Eigen::AngleAxisf init_rotation_z(guess_pose.yaw, Eigen::Vector3f::UnitZ());
  Eigen::Translation3f init_translation(guess_pose.x, guess_pose.y, guess_pose.z);
  Eigen::Matrix4f init_guess =
      (init_translation * init_rotation_z * init_rotation_y * init_rotation_x).matrix() * tf_btol;
  
  t1 = std::chrono::system_clock::now();
#ifdef USE_FAST_PCL
  pcl::PointCloud<pcl::PointXYZI>::Ptr output_cloud(new pcl::PointCloud<pcl::PointXYZI>);
  ndt.omp_align(*output_cloud, init_guess);
  t_localizer = ndt.getFinalTransformation();
  has_converged = ndt.hasConverged();
  fitness_score = ndt.omp_getFitnessScore();
  final_num_iteration = ndt.getFinalNumIteration();
#elif defined USE_GPU_PCL
  gpu_ndt.align(init_guess);
  t_localizer = gpu_ndt.getFinalTransformation();
  has_converged = gpu_ndt.hasConverged();
  fitness_score = gpu_ndt.getFitnessScore();
  final_num_iteration = gpu_ndt.getFinalNumIteration();
#else
  pcl::PointCloud<pcl::PointXYZI>::Ptr output_cloud(new pcl::PointCloud<pcl::PointXYZI>);
  ndt.align(*output_cloud, init_guess);
  t_localizer = ndt.getFinalTransformation();
  has_converged = ndt.hasConverged();
  fitness_score = ndt.getFitnessScore();
  final_num_iteration = ndt.getFinalNumIteration();
#endif
  t2 = std::chrono::system_clock::now();
  double ndt_align_time = std::chrono::duration_cast<std::chrono::microseconds>(t2 - t1).count() / 1000.0;
  
  t_base_link = t_localizer * tf_ltob;
  
  tf::Matrix3x3 mat_l, mat_b;

  mat_l.setValue(static_cast<double>(t_localizer(0, 0)), static_cast<double>(t_localizer(0, 1)),
                 static_cast<double>(t_localizer(0, 2)), static_cast<double>(t_localizer(1, 0)),
                 static_cast<double>(t_localizer(1, 1)), static_cast<double>(t_localizer(1, 2)),
                 static_cast<double>(t_localizer(2, 0)), static_cast<double>(t_localizer(2, 1)),
                 static_cast<double>(t_localizer(2, 2)));

  mat_b.setValue(static_cast<double>(t_base_link(0, 0)), static_cast<double>(t_base_link(0, 1)),
                 static_cast<double>(t_base_link(0, 2)), static_cast<double>(t_base_link(1, 0)),
                 static_cast<double>(t_base_link(1, 1)), static_cast<double>(t_base_link(1, 2)),
                 static_cast<double>(t_base_link(2, 0)), static_cast<double>(t_base_link(2, 1)),
                 static_cast<double>(t_base_link(2, 2)));

  // Update localizer_pose.
  localizer_pose.x = t_localizer(0, 3);
  localizer_pose.y = t_localizer(1, 3);
  localizer_pose.z = t_localizer(2, 3);
  mat_l.getRPY(localizer_pose.roll, localizer_pose.pitch, localizer_pose.yaw, 1);

  // Update ndt_pose.
  ndt_pose.x = t_base_link(0, 3);
  ndt_pose.y = t_base_link(1, 3);
  ndt_pose.z = t_base_link(2, 3);
  mat_b.getRPY(ndt_pose.roll, ndt_pose.pitch, ndt_pose.yaw, 1);

  current_pose.x = ndt_pose.x;
  current_pose.y = ndt_pose.y;
  current_pose.z = ndt_pose.z;
  current_pose.roll = ndt_pose.roll;
  current_pose.pitch = ndt_pose.pitch;
  current_pose.yaw = ndt_pose.yaw;
  pcl::getTransformation(current_pose.x, current_pose.y, current_pose.z,
                         current_pose.roll, current_pose.pitch, current_pose.yaw,
                         current_pose_tf);

  // transform.setOrigin(tf::Vector3(current_pose.x, current_pose.y, current_pose.z));
  // q.setRPY(current_pose.roll, current_pose.pitch, current_pose.yaw);
  // transform.setRotation(q); // duplicate??

  // Calculate the offset (curren_pos - previous_pos)
  diff_x = current_pose.x - previous_pose.x;
  diff_y = current_pose.y - previous_pose.y;
  diff_z = current_pose.z - previous_pose.z;
  diff_roll = current_pose.roll - previous_pose.roll;
  diff_pitch = current_pose.pitch - previous_pose.pitch;
  diff_yaw = current_pose.yaw - previous_pose.yaw;
  diff = sqrt(diff_x * diff_x + diff_y * diff_y + diff_z * diff_z);
  
  // Calculate the shift between added_pos and current_pos
  double t_shift = sqrt(pow(current_pose.x - added_pose.x, 2.0) + pow(current_pose.y - added_pose.y, 2.0));
  double R_shift = std::fabs(current_pose.yaw - added_pose.yaw);
  t1 = std::chrono::system_clock::now();
  pcl::transformPointCloud(*scan_ptr, *transformed_scan_ptr, t_localizer);
  if(t_shift >= min_add_scan_shift || R_shift >= min_add_scan_yaw_diff)
  {
#ifdef MY_EXTRACT_SCANPOSE

    // outputing into csv
    csv_stream << add_scan_number << "," << input->header.seq << "," << current_scan_time.sec << "," << current_scan_time.nsec << ","
               << localizer_pose.x << "," << localizer_pose.y << "," << localizer_pose.z << ","
               << localizer_pose.roll << "," << localizer_pose.pitch << "," << localizer_pose.yaw
               << std::endl;
#endif // MY_EXTRACT_SCANPOSE

    add_new_scan(*transformed_scan_ptr);
    add_scan_number++;
    added_pose.x = current_pose.x;
    added_pose.y = current_pose.y;
    added_pose.z = current_pose.z;
    added_pose.roll = current_pose.roll;
    added_pose.pitch = current_pose.pitch;
    added_pose.yaw = current_pose.yaw;
    isMapUpdate = true;
  }
#ifdef MY_EXTRACT_SCANPOSE
  else
  {
    // outputing into csv, with add_scan_number = 0
    csv_stream << 0 << "," << input->header.seq << "," << current_scan_time.sec << "," << current_scan_time.nsec << ","
               << localizer_pose.x << "," << localizer_pose.y << "," << localizer_pose.z << ","
               << localizer_pose.roll << "," << localizer_pose.pitch << "," << localizer_pose.yaw
               << std::endl;
  }
#endif // MY_EXTRACT_SCANPOSE
  t2 = std::chrono::system_clock::now();
  double ndt_keyscan_time = std::chrono::duration_cast<std::chrono::microseconds>(t2 - t1).count() / 1000.0;

  // do a bit of evaluation on correctLidarScan() here
  #ifdef CORRECT_SCAN_DEBUG
  pose poseA, poseB;
  double secA = secs;
  double secB = (current_scan_time - previous_scan_time).toSec();
  pcl::getTranslationAndEulerAngles(relative_pose_tf, // the rel_pose_tf used just now
                                    poseA.x, poseA.y, poseA.z,
                                    poseA.roll, poseA.pitch, poseA.yaw);
  pcl::getTranslationAndEulerAngles(previous_pose_tf.inverse() * current_pose_tf, // this should be the next rel_pose_tf
                                    poseB.x, poseB.y, poseB.z,
                                    poseB.roll, poseB.pitch, poseB.yaw);
  // poseA.x = poseA.x/secA; poseA.y = poseA.y/secA; poseA.z = poseA.z/secA;
  // poseA.roll = poseA.roll/secA; poseA.pitch = poseA.pitch/secA; poseA.yaw = poseA.yaw/secA;
  // poseB.x = poseB.x/secB; poseB.y = poseB.y/secB; poseB.z = poseB.z/secB;
  // poseB.roll = poseB.roll/secB; poseB.pitch = poseB.pitch/secB; poseB.yaw = poseB.yaw/secB;
  std::cout << "-----------------------------------------------------------------\n";
  std::cout << "Correct scan evaluation: \n";
  std::cout << "(" << (poseA.x - poseB.x) << ","
                   << (poseA.y - poseB.y) << ","
                   << (poseA.z - poseB.z) << ","
                   << (poseA.roll - poseB.roll) << ","
                   << (poseA.pitch - poseB.pitch) << ","
                   << (poseA.yaw - poseB.yaw) << ")" << std::endl;
  #endif // CORRECT_SCAN_DEBUG

  transform.setOrigin(tf::Vector3(current_pose.x, current_pose.y, current_pose.z));
  q.setRPY(current_pose.roll, current_pose.pitch, current_pose.yaw);
  transform.setRotation(q);
  br.sendTransform(tf::StampedTransform(transform, current_scan_time, "map", "base_link"));

  scan_duration = current_scan_time - previous_scan_time;
  secs = scan_duration.toSec();

  // current_velocity.x = diff_x / secs;
  // current_velocity.y = diff_y / secs;
  // current_velocity.z = diff_z / secs;
  // current_velocity.roll = diff_roll / secs;
  // current_velocity.pitch = diff_pitch / secs;
  // current_velocity.yaw = diff_yaw / secs;
  relative_pose_tf = previous_pose_tf.inverse() * current_pose_tf;

  // Update position and posture. current_pos -> previous_pos
  previous_pose.x = current_pose.x;
  previous_pose.y = current_pose.y;
  previous_pose.z = current_pose.z;
  previous_pose.roll = current_pose.roll;
  previous_pose.pitch = current_pose.pitch;
  previous_pose.yaw = current_pose.yaw;
  previous_pose_tf = current_pose_tf;

  previous_scan_time.sec = current_scan_time.sec;
  previous_scan_time.nsec = current_scan_time.nsec;

  sensor_msgs::PointCloud2::Ptr map_msg_ptr(new sensor_msgs::PointCloud2);
  pcl::toROSMsg(*local_map_ptr, *map_msg_ptr);
  ndt_map_pub.publish(*map_msg_ptr);

  sensor_msgs::PointCloud2::Ptr scan_msg_ptr(new sensor_msgs::PointCloud2);
  transformed_scan_ptr->header.frame_id = "map";
  pcl::toROSMsg(*transformed_scan_ptr, *scan_msg_ptr);
  current_scan_pub.publish(*scan_msg_ptr);

  pcl::PointCloud<pcl::PointXYZI>::Ptr transformed_tmp_ptr(new pcl::PointCloud<pcl::PointXYZI>());
  #ifdef LIMIT_HEIGHT
  pcl::transformPointCloud(src, *transformed_tmp_ptr, t_localizer);
  #else
  pcl::transformPointCloud(*filtered_scan_ptr, *transformed_tmp_ptr, t_localizer);
  #endif // LIMIT_HEIGHT
  sensor_msgs::PointCloud2::Ptr tmp_msg_ptr(new sensor_msgs::PointCloud2);
  pcl::toROSMsg(*transformed_tmp_ptr, *tmp_msg_ptr);
  tmp_msg_ptr->header.frame_id = "map";
  original_scan_pub.publish(*tmp_msg_ptr);

  std::cout << "-----------------------------------------------------------------\n";
  std::cout << "Sequence number: " << input->header.seq << "\n";
  std::cout << "Number of scan points: " << scan_ptr->size() << " points.\n";
  std::cout << "Number of filtered scan points: " << filtered_scan_ptr->size() << " points.\n";
  std::cout << "Local map: " << local_map.points.size() << " points.\n";
  std::cout << "NDT has converged: " << has_converged << "\n";
  std::cout << "Fitness score: " << fitness_score << "\n";
  std::cout << "Number of iteration: " << final_num_iteration << "\n";
  // std::cout << "Guessed posed: " << "\n";
  // std::cout << "(" << guess_pose.x << ", " << guess_pose.y << ", " << guess_pose.z << ", " << guess_pose.roll
  //           << ", " << guess_pose.pitch << ", " << guess_pose.yaw << ")\n";
  std::cout << "(x,y,z,roll,pitch,yaw):" << "\n";
  std::cout << "(" << current_pose.x << ", " << current_pose.y << ", " << current_pose.z << ", " << current_pose.roll
            << ", " << current_pose.pitch << ", " << current_pose.yaw << ")\n";
  // std::cout << "Transformation Matrix:\n";
  // std::cout << t_localizer << "\n";
  std::cout << "Translation shift: " << t_shift << "\n";
  std::cout << "Rotation (yaw) shift: " << R_shift << "\n";
  std::cout << "Update target map took: " << ndt_update_time << "ms.\n";
  std::cout << "NDT matching took: " << ndt_align_time << "ms.\n";
  std::cout << "Updating map took: " << ndt_keyscan_time << "ms.\n";
  std::cout << "-----------------------------------------------------------------" << std::endl;
}

static void map_maintenance_callback(pose local_pose)
{
  // Get local_key
  local_key = {int(floor(local_pose.x / TILE_WIDTH)),  // .x
               int(floor(local_pose.y / TILE_WIDTH))}; // .y
  // local_key.x = int(floor(local_pose.x / TILE_WIDTH));
  // local_key.y = int(floor(local_pose.y / TILE_WIDTH));

  // Only update local_map through world_map only if local_key changes
  if(local_key != previous_key)
  {
    std::lock_guard<std::mutex> lck(mtx);
    // Get local_map, a 3x3 tile map with the center being the local_key
    local_map.clear();
    Key tmp_key;
    for(int x = local_key.x - 2, x_max = local_key.x + 2; x <= x_max; x++)
      for(int y = local_key.y - 2, y_max = local_key.y + 2; y <= y_max; y++)
      {
        tmp_key.x = x;
        tmp_key.y = y;
        local_map += world_map[tmp_key];
      }

    // Update key
    previous_key = local_key;
  }
}

void mySigintHandler(int sig) // Publish the map/final_submap if node is terminated
{
  char buffer[100];
  std::strftime(buffer, 100, "%Y%b%d_%H%M", pnow);
  std::string filename = _output_directory + "ndt_" + std::string(buffer) + ".pcd";

  // Write a config file
  char cbuffer[100];
  std::ofstream config_stream;
  std::strftime(cbuffer, 100, "%b%d-%H%M", pnow);
  std::string config_file = "config@" + std::string(cbuffer) + ".txt";
  config_stream.open(_output_directory + config_file);
  std::strftime(cbuffer, 100, "%c", pnow);

  std::time_t process_end = std::time(NULL);
  double process_duration = difftime(process_end, process_begin); // calculate processing duration
  int process_hr = int(process_duration / 3600);
  int process_min = int((process_duration - process_hr * 3600) / 60);
  double process_sec = process_duration - process_hr * 3600 - process_min * 60;

  config_stream << "Created @ " << std::string(cbuffer) << std::endl;
  config_stream << "Map: " << _bag_file << std::endl;
  config_stream << "Start time: " << _start_time << std::endl;
  config_stream << "Play duration: " << (_play_duration < 0 ? "all" : std::to_string(_play_duration)) << std::endl;
  config_stream << "Corrected end scan pose: ?\n" << std::endl;
  config_stream << "\nResolution: " << ndt_res << std::endl;
  config_stream << "Step Size: " << step_size << std::endl;
  config_stream << "Transformation Epsilon: " << trans_eps << std::endl;
  config_stream << "Max Iteration Number: " << max_iter << std::endl;
  config_stream << "\nLeaf Size: " << voxel_leaf_size << std::endl;
  config_stream << "Minimum Scan Range: " << min_scan_range << std::endl;
  config_stream << "Minimum Add Scan Shift: " << min_add_scan_shift << std::endl;
  config_stream << "Minimum Add Scan Yaw Change: " << min_add_scan_yaw_diff << std::endl;
#ifdef TILE_WIDTH
  config_stream << "Tile-map type used. Size of each tile: " 
                << TILE_WIDTH << "x" << TILE_WIDTH << std::endl;
  config_stream << "Size of local map: 5 tiles x 5 tiles." << std::endl;
#endif // TILE_WIDTH
  config_stream << "Time taken: " << process_hr << " hr "
                                  << process_min << " min "
                                  << process_sec << " sec" << std::endl;

  std::cout << "-----------------------------------------------------------------\n";
  std::cout << "Writing the last map to pcd file before shutting down node..." << std::endl;

  pcl::PointCloud<pcl::PointXYZI> last_map;
  for (auto& item: world_map) 
    last_map += item.second;

  last_map.header.frame_id = "map";
  pcl::io::savePCDFileBinary(filename, last_map);
  std::cout << "Saved " << last_map.points.size() << " data points to " << filename << ".\n";
  std::cout << "-----------------------------------------------------------------" << std::endl;
  std::cout << "Done. Node will now shutdown." << std::endl;

  // All the default sigint handler does is call shutdown()
  ros::shutdown();
}

int main(int argc, char** argv)
{
  previous_pose.x = 0.0;
  previous_pose.y = 0.0;
  previous_pose.z = 0.0;
  previous_pose.roll = 0.0;
  previous_pose.pitch = 0.0;
  previous_pose.yaw = 0.0;

  ndt_pose.x = 0.0;
  ndt_pose.y = 0.0;
  ndt_pose.z = 0.0;
  ndt_pose.roll = 0.0;
  ndt_pose.pitch = 0.0;
  ndt_pose.yaw = 0.0;

  current_pose.x = 0.0;
  current_pose.y = 0.0;
  current_pose.z = 0.0;
  current_pose.roll = 0.0;
  current_pose.pitch = 0.0;
  current_pose.yaw = 0.0;

  guess_pose.x = 0.0;
  guess_pose.y = 0.0;
  guess_pose.z = 0.0;
  guess_pose.roll = 0.0;
  guess_pose.pitch = 0.0;
  guess_pose.yaw = 0.0;

  added_pose.x = 0.0;
  added_pose.y = 0.0;
  added_pose.z = 0.0;
  added_pose.roll = 0.0;
  added_pose.pitch = 0.0;
  added_pose.yaw = 0.0;

  diff = 0.0;
  diff_x = 0.0;
  diff_y = 0.0;
  diff_z = 0.0;
  diff_yaw = 0.0;

  // current_velocity.x = 0;
  // current_velocity.y = 0;
  // current_velocity.z = 0;

  pcl::getTransformation(0, 0, 0, 0, 0, 0, current_pose_tf);
  pcl::getTransformation(0, 0, 0, 0, 0, 0, previous_pose_tf);
  pcl::getTransformation(0, 0, 0, 0, 0, 0, relative_pose_tf);

  ros::init(argc, argv, "ndt_mapping", ros::init_options::NoSigintHandler);
  ros::NodeHandle private_nh("~");

  // get setting parameters in the launch files
  private_nh.getParam("bag_file", _bag_file);
  private_nh.getParam("start_time", _start_time);
  private_nh.getParam("play_duration", _play_duration);
  private_nh.getParam("namespace", _namespace);
  private_nh.getParam("output_directory", _output_directory);

  private_nh.getParam("resolution", ndt_res);
  private_nh.getParam("step_size", step_size);  
  private_nh.getParam("transformation_epsilon", trans_eps);
  private_nh.getParam("max_iteration", max_iter);

  private_nh.getParam("voxel_leaf_size", voxel_leaf_size);
  private_nh.getParam("min_scan_range", min_scan_range);
  private_nh.getParam("min_add_scan_shift", min_add_scan_shift);
  private_nh.getParam("min_add_scan_yaw_diff", min_add_scan_yaw_diff);

  private_nh.getParam("tf_x", _tf_x);
  private_nh.getParam("tf_y", _tf_y);
  private_nh.getParam("tf_z", _tf_z);
  private_nh.getParam("tf_roll", _tf_roll);
  private_nh.getParam("tf_pitch", _tf_pitch);
  private_nh.getParam("tf_yaw", _tf_yaw);

  std::cout << "\nNDT Mapping Parameters:" << std::endl;
  std::cout << "bag_file: " << _bag_file << std::endl;
  std::cout << "start_time: " << _start_time << std::endl;
  std::cout << "play_duration: " << _play_duration << std::endl;
  std::cout << "namespace: " << (_namespace.size() > 0 ? _namespace : "N/A") << std::endl;
  std::cout << "ndt_res: " << ndt_res << std::endl;
  std::cout << "step_size: " << step_size << std::endl;
  std::cout << "trans_epsilon: " << trans_eps << std::endl;
  std::cout << "max_iter: " << max_iter << std::endl;
  std::cout << "voxel_leaf_size: " << voxel_leaf_size << std::endl;
  std::cout << "min_scan_range: " << min_scan_range << std::endl;
  std::cout << "min_add_scan_shift: " << min_add_scan_shift << std::endl;
  std::cout << "min_add_scan_yaw_diff: " << min_add_scan_yaw_diff << std::endl;
  std::cout << "(tf_x,tf_y,tf_z,tf_roll,tf_pitch,tf_yaw): (" << _tf_x << ", " << _tf_y << ", " << _tf_z << ", "
            << _tf_roll << ", " << _tf_pitch << ", " << _tf_yaw << ")\n" << std::endl;

  ros::NodeHandle nh;
  signal(SIGINT, mySigintHandler);

  if(_namespace.size() > 0)
  {
    ndt_map_pub = nh.advertise<sensor_msgs::PointCloud2>(_namespace + "/local_map", 1000, true);
    current_scan_pub = nh.advertise<sensor_msgs::PointCloud2>(_namespace + "/current_scan", 1, true);
    original_scan_pub = nh.advertise<sensor_msgs::PointCloud2>(_namespace + "/source_scan", 1, true);
  }
  else
  {
    ndt_map_pub = nh.advertise<sensor_msgs::PointCloud2>("/local_map", 1000, true);
    current_scan_pub = nh.advertise<sensor_msgs::PointCloud2>("/current_scan", 1, true);
    original_scan_pub = nh.advertise<sensor_msgs::PointCloud2>("/source_scan", 1, true);
  }

  try
  {
    mkdir(_output_directory.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
  }
  catch(...)
  {
    std::cout << "ERROR: Failed to access directory:" << _output_directory << std::endl;
    return -1;
  }

#ifdef USE_GPU_PCL
  gpu_ndt.setTransformationEpsilon(trans_eps);
  gpu_ndt.setStepSize(step_size);
  gpu_ndt.setResolution(ndt_res);
  gpu_ndt.setMaximumIterations(max_iter);
#else
  ndt.setTransformationEpsilon(trans_eps);
  ndt.setStepSize(step_size);
  ndt.setResolution(ndt_res);
  ndt.setMaximumIterations(max_iter);
#endif

  Eigen::Translation3f tl_btol(_tf_x, _tf_y, _tf_z);                 // tl: translation
  Eigen::AngleAxisf rot_x_btol(_tf_roll, Eigen::Vector3f::UnitX());  // rot: rotation
  Eigen::AngleAxisf rot_y_btol(_tf_pitch, Eigen::Vector3f::UnitY());
  Eigen::AngleAxisf rot_z_btol(_tf_yaw, Eigen::Vector3f::UnitZ());
  tf_btol = (tl_btol * rot_z_btol * rot_y_btol * rot_x_btol).matrix();

  Eigen::Translation3f tl_ltob((-1.0) * _tf_x, (-1.0) * _tf_y, (-1.0) * _tf_z);  // tl: translation
  Eigen::AngleAxisf rot_x_ltob((-1.0) * _tf_roll, Eigen::Vector3f::UnitX());     // rot: rotation
  Eigen::AngleAxisf rot_y_ltob((-1.0) * _tf_pitch, Eigen::Vector3f::UnitY());
  Eigen::AngleAxisf rot_z_ltob((-1.0) * _tf_yaw, Eigen::Vector3f::UnitZ());
  tf_ltob = (tl_ltob * rot_z_ltob * rot_y_ltob * rot_x_ltob).matrix();

  local_map.header.frame_id = "map";

#ifdef MY_EXTRACT_SCANPOSE // map_pose.csv
  csv_stream.open(_output_directory + csv_filename);
  csv_stream << "key,sequence,sec,nsec,x,y,z,roll,pitch,yaw" << std::endl;
#endif // MY_EXTRACT_SCANPOSE

  // Open bagfile with topics, timestamps indicated
  std::cout << "Loading " << _bag_file << std::endl;
  rosbag::Bag bag(_bag_file, rosbag::bagmode::Read);
  std::vector<std::string> reading_topics;
    reading_topics.push_back(std::string("/points_raw"));
  ros::Time rosbag_start_time = ros::TIME_MAX;
  ros::Time rosbag_stop_time = ros::TIME_MIN;
  if(_play_duration <= 0)
  {
    rosbag::View tmp_view(bag);
    rosbag_start_time = tmp_view.getBeginTime() + ros::Duration(_start_time);
    rosbag_stop_time = ros::TIME_MAX;
  }
  else
  {
    rosbag::View tmp_view(bag);
    rosbag_start_time = tmp_view.getBeginTime() + ros::Duration(_start_time);
    ros::Duration sim_duration(_play_duration);
    rosbag_stop_time = rosbag_start_time + sim_duration;
  }
  rosbag::View view(bag, rosbag::TopicQuery(reading_topics), rosbag_start_time, rosbag_stop_time);
  const int msg_size = view.size();
  int msg_pos = 0;

  // Looping, processing messages in bag file
  std::chrono::time_point<std::chrono::system_clock> t1, t2, t3;
  std::cout << "Finished preparing bagfile. Starting mapping..." << std::endl;
  std::cout << "Note: if the mapping does not start immediately, check the subscribed topic names.\n" << std::endl;
  foreach(rosbag::MessageInstance const message, view)
  {
    sensor_msgs::PointCloud2::ConstPtr input_cloud = message.instantiate<sensor_msgs::PointCloud2>();
    if(input_cloud == NULL)
    {
      std::cout << "No input PointCloud available. Waiting..." << std::endl;
      continue;
    }

    // Global callback to call scans process and submap process
    t1 = std::chrono::system_clock::now();
    map_maintenance_callback(current_pose);
    t2 = std::chrono::system_clock::now();
    ndt_mapping_callback(input_cloud);
    t3 = std::chrono::system_clock::now();
    // #pragma omp parallel sections
    // {
    //   #pragma omp section
    //   {
    //     ndt_mapping_callback(input_cloud);
    //   }
    //   #pragma omp section
    //   {
    //     std::cout << "current_pose: (" << current_pose.x << "," << current_pose.y << "," << current_pose.z << ","
    //                                    << current_pose.roll << "," << current_pose.pitch << "," << current_pose.yaw << std::endl;
    //     //using current_pose as local_pose to get local_map
    //     map_maintenance_callback(current_pose);
    //   }
    // }
    msg_pos++;
    std::cout << "---Number of key scans: " << add_scan_number << "\n";
    std::cout << "---Processed: " << msg_pos << "/" << msg_size << "\n";
    std::cout << "---Get local map took: " << std::chrono::duration_cast<std::chrono::nanoseconds>(t2 - t1).count() / 1.0 << "ns.\n";
    std::cout << "---NDT Mapping took: " << std::chrono::duration_cast<std::chrono::microseconds>(t3 - t2).count() / 1000.0 << "ms."<< std::endl;
  }
  bag.close();
  std::cout << "Finished processing bag file." << std::endl;

  mySigintHandler(0);

  return 0;
}