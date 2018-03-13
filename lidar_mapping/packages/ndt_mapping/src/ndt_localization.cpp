// Basic libs
#include <chrono>
#include <fstream>
#include <iostream>
#include <sstream>
#include <mutex>
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
// #include <lidar_pcl/data_types.h>

// Here are the functions I wrote. De-comment to use
#define TILE_WIDTH 35
#define MY_EXTRACT_SCANPOSE // do not use this, this is to extract scans and poses to close loop

#ifdef MY_EXTRACT_SCANPOSE
std::ofstream csv_stream;
std::string csv_filename = "map_pose.csv";
#endif // MY_EXTRACT_SCANPOSE

typedef pcl::PointXYZI Point_t;
typedef pcl::PointCloud<Point_t> Cloud_t;

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

// global variables
static pose previous_pose, guess_pose, current_pose, ndt_pose, localizer_pose;
static Eigen::Affine3d current_pose_tf, previous_pose_tf, relative_pose_tf;
static ros::Publisher local_map_pub, current_scan_pub, original_scan_pub;

static ros::Time current_scan_time;
static ros::Time previous_scan_time;
static ros::Duration scan_duration;

static double diff, diff_x, diff_y, diff_z, diff_roll, diff_pitch, diff_yaw; // current_pose - previous_pose
static double secs = 0.100085; // scan duration
// static velocity current_velocity;

// static pcl::PointCloud<pcl::PointXYZI> world_map;
static std::unordered_map<Key, Cloud_t> world_map;
static Cloud_t local_map;
static Key previous_key;
static std::mutex mtx;

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
static std::string _map;
static std::string _output_directory;
static std::string _namespace;

static double _tf_x, _tf_y, _tf_z, _tf_roll, _tf_pitch, _tf_yaw;
static Eigen::Matrix4f tf_btol, tf_ltob;

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

static void update_local_map(pose local_pose)
{
  // Get local_key
  Key local_key = {int(floor(local_pose.x / TILE_WIDTH)),  // .x
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

    // Update key and map
    previous_key = local_key;
    Cloud_t::Ptr local_map_ptr(new Cloud_t(local_map));
  #ifdef USE_GPU_PCL
    gpu_ndt.setInputTarget(local_map_ptr);
  #else
    ndt.setInputTarget(local_map_ptr);
  #endif

    sensor_msgs::PointCloud2 local_map_msg;
    pcl::toROSMsg(*local_map_ptr, local_map_msg);
    local_map_msg.header.frame_id = "map";
    local_map_pub.publish(local_map_msg);
    ROS_INFO("Local map changed.");
  }
}

static void ndt_mapping_callback(const sensor_msgs::PointCloud2::ConstPtr& input)
{
  std::chrono::time_point<std::chrono::system_clock> t1 = std::chrono::system_clock::now();
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
  // lidar_pcl::fromROSMsg(*input, tmp); // dont enable

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

  lidar_pcl::motionUndistort(scan, relative_pose_tf);
  pcl::PointCloud<pcl::PointXYZI>::Ptr scan_ptr(new pcl::PointCloud<pcl::PointXYZI>(scan));

  // Apply voxelgrid filter
  pcl::VoxelGrid<pcl::PointXYZI> voxel_grid_filter;
  voxel_grid_filter.setLeafSize(voxel_leaf_size, voxel_leaf_size, voxel_leaf_size);
  voxel_grid_filter.setInputCloud(scan_ptr);
  voxel_grid_filter.filter(*filtered_scan_ptr);

#ifdef USE_GPU_PCL
  gpu_ndt.setInputSource(filtered_scan_ptr);
#else
  ndt.setInputSource(filtered_scan_ptr);
#endif

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

  // Calculate the offset (curren_pos - previous_pos)
  diff_x = current_pose.x - previous_pose.x;
  diff_y = current_pose.y - previous_pose.y;
  diff_z = current_pose.z - previous_pose.z;
  diff_roll = current_pose.roll - previous_pose.roll;
  diff_pitch = current_pose.pitch - previous_pose.pitch;
  diff_yaw = current_pose.yaw - previous_pose.yaw;
  diff = sqrt(diff_x * diff_x + diff_y * diff_y + diff_z * diff_z);
  
  pcl::transformPointCloud(*scan_ptr, *transformed_scan_ptr, t_localizer);

#ifdef MY_EXTRACT_SCANPOSE
  // outputing into csv, with add_scan_number = 0
  csv_stream << 0 << "," << input->header.seq << "," << current_scan_time.sec << "," << current_scan_time.nsec << ","
              << localizer_pose.x << "," << localizer_pose.y << "," << localizer_pose.z << ","
              << localizer_pose.roll << "," << localizer_pose.pitch << "," << localizer_pose.yaw
              << std::endl;
#endif // MY_EXTRACT_SCANPOSE

  transform.setOrigin(tf::Vector3(current_pose.x, current_pose.y, current_pose.z));
  q.setRPY(current_pose.roll, current_pose.pitch, current_pose.yaw);
  transform.setRotation(q);
  br.sendTransform(tf::StampedTransform(transform, current_scan_time, "map", "base_link"));

  scan_duration = current_scan_time - previous_scan_time;
  secs = scan_duration.toSec();

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

  sensor_msgs::PointCloud2::Ptr scan_msg_ptr(new sensor_msgs::PointCloud2);
  transformed_scan_ptr->header.frame_id = "map";
  pcl::toROSMsg(*transformed_scan_ptr, *scan_msg_ptr);
  current_scan_pub.publish(*scan_msg_ptr);

  pcl::PointCloud<pcl::PointXYZI>::Ptr transformed_tmp_ptr(new pcl::PointCloud<pcl::PointXYZI>());
  pcl::transformPointCloud(*filtered_scan_ptr, *transformed_tmp_ptr, t_localizer);
  sensor_msgs::PointCloud2::Ptr tmp_msg_ptr(new sensor_msgs::PointCloud2);
  pcl::toROSMsg(*transformed_tmp_ptr, *tmp_msg_ptr);
  tmp_msg_ptr->header.frame_id = "map";
  original_scan_pub.publish(*tmp_msg_ptr);

  std::chrono::time_point<std::chrono::system_clock> t2 = std::chrono::system_clock::now();
  std::cout << "-----------------------------------------------------------------\n";
  std::cout << "Sequence number: " << input->header.seq << "\n";
  std::cout << "Number of scan points: " << scan_ptr->size() << " points.\n";
  std::cout << "Number of filtered scan points: " << filtered_scan_ptr->size() << " points.\n";
  std::cout << "NDT has converged: " << has_converged << "\n";
  std::cout << "Fitness score: " << fitness_score << "\n";
  std::cout << "Number of iteration: " << final_num_iteration << "\n";
  // std::cout << "Guessed posed: pitch << ", " << guess_pose.yaw << ")\n";
  std::cout << "(x,y,z,roll,pitch,yaw):" << "\n";
  std::cout << "(" << current_pose.x << ", " << current_pose.y << ", " << current_pose.z << ", " << current_pose.roll
            << ", " << current_pose.pitch << ", " << current_pose.yaw << ")\n";
  // std::cout << "Transformation Matrix:\n";
  // std::cout << t_localizer << "\n";
  std::cout << "Callback took: " << std::chrono::duration_cast<std::chrono::microseconds>(t2 - t1).count() / 1000.0 << "ms.\n";
  std::cout << "-----------------------------------------------------------------" << std::endl;

  update_local_map(current_pose);
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
  config_stream << "Map: " << _map << std::endl;
  config_stream << "Corrected end scan pose: ?\n" << std::endl;
  config_stream << "\nResolution: " << ndt_res << std::endl;
  config_stream << "Step Size: " << step_size << std::endl;
  config_stream << "Transformation Epsilon: " << trans_eps << std::endl;
  config_stream << "Max Iteration Number: " << max_iter << std::endl;
  config_stream << "\nLeaf Size: " << voxel_leaf_size << std::endl;
  config_stream << "Minimum Scan Range: " << min_scan_range << std::endl;
  config_stream << "Minimum Add Scan Shift: " << min_add_scan_shift << std::endl;
  config_stream << "Minimum Add Scan Yaw Change: " << min_add_scan_yaw_diff << std::endl;
  config_stream << "Time taken: " << process_hr << " hr "
                                  << process_min << " min "
                                  << process_sec << " sec" << std::endl;

  std::cout << "-----------------------------------------------------------------\n";
  std::cout << "Done. Node will now shutdown." << std::endl;

  // All the default sigint handler does is call shutdown()
  ros::shutdown();
}

int main(int argc, char** argv)
{

  pcl::getTransformation(0, 0, 0, 0, 0, 0, current_pose_tf);
  pcl::getTransformation(0, 0, 0, 0, 0, 0, previous_pose_tf);
  pcl::getTransformation(0, 0, 0, 0, 0, 0, relative_pose_tf);

  ros::init(argc, argv, "ndt_localization", ros::init_options::NoSigintHandler);
  ros::NodeHandle private_nh("~");

  // get setting parameters in the launch files
  private_nh.getParam("map", _map);
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

  private_nh.getParam("initial_x", guess_pose.x);
  private_nh.getParam("initial_y", guess_pose.y);
  private_nh.getParam("initial_z", guess_pose.z);
  private_nh.getParam("initial_roll", guess_pose.roll);
  private_nh.getParam("initial_pitch", guess_pose.pitch);
  private_nh.getParam("initial_yaw", guess_pose.yaw);

  private_nh.getParam("tf_x", _tf_x);
  private_nh.getParam("tf_y", _tf_y);
  private_nh.getParam("tf_z", _tf_z);
  private_nh.getParam("tf_roll", _tf_roll);
  private_nh.getParam("tf_pitch", _tf_pitch);
  private_nh.getParam("tf_yaw", _tf_yaw);

  // guess_pose.x -= _tf_x;
  // guess_pose.y -= _tf_y;
  // guess_pose.z -= _tf_z;

  previous_pose.y = guess_pose.x;
  previous_pose.x = guess_pose.y;
  previous_pose.z = guess_pose.z;
  previous_pose.roll = guess_pose.roll;
  previous_pose.pitch = guess_pose.pitch;
  previous_pose.yaw = guess_pose.yaw;

  ndt_pose.x = 0.0;
  ndt_pose.y = 0.0;
  ndt_pose.z = 0.0;
  ndt_pose.roll = 0.0;
  ndt_pose.pitch = 0.0;
  ndt_pose.yaw = 0.0;

  current_pose.x = guess_pose.x;
  current_pose.y = guess_pose.y;
  current_pose.z = guess_pose.z;
  current_pose.roll = guess_pose.roll;
  current_pose.pitch = guess_pose.pitch;
  current_pose.yaw = guess_pose.yaw;

  diff = 0.0;
  diff_x = 0.0;
  diff_y = 0.0;
  diff_z = 0.0;
  diff_yaw = 0.0;

  std::cout << "\nNDT Mapping Parameters:" << std::endl;
  std::cout << "map: " << _map << std::endl;
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
  ros::Subscriber pcl_sub = nh.subscribe("/points_raw", 1000, ndt_mapping_callback);

  if(_namespace.size() > 0)
  {
    local_map_pub = nh.advertise<sensor_msgs::PointCloud2>(_namespace + "/local_map", 10, true);
    current_scan_pub = nh.advertise<sensor_msgs::PointCloud2>(_namespace + "/current_scan", 10, true);
    original_scan_pub = nh.advertise<sensor_msgs::PointCloud2>(_namespace + "/source_scan", 10, true);
  }
  else
  {
    local_map_pub = nh.advertise<sensor_msgs::PointCloud2>(_namespace + "/local_map", 10, true);
    current_scan_pub = nh.advertise<sensor_msgs::PointCloud2>("/current_scan", 10, true);
    original_scan_pub = nh.advertise<sensor_msgs::PointCloud2>("/source_scan", 10, true);
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

  // Get target world map
  if(pcl::io::loadPCDFile<pcl::PointXYZI>(_map, local_map) == -1)
  {
    std::cout << "ERROR: Couldn't read " << _map << "." << std::endl;
    return(-1);
  }
  else
    std::cout << _map << " loaded." << std::endl;

  for(pcl::PointCloud<pcl::PointXYZI>::const_iterator item = local_map.begin(); item < local_map.end(); item++)
  {
    // Get 2D point
    Key key;
    key.x = int(floor(item->x / TILE_WIDTH));
    key.y = int(floor(item->y / TILE_WIDTH));

    world_map[key].push_back(*item);
  }

  // Update local_map for the first time
  previous_key = Key{int(floor(current_pose.x / TILE_WIDTH)), int(floor(current_pose.y / TILE_WIDTH))};
  local_map.clear();
  for(int x = previous_key.x - 2, x_max = previous_key.x + 2; x <= x_max; x++)
    for(int y = previous_key.y - 2, y_max = previous_key.y + 2; y <= y_max; y++)
    {
      Key tmp_key = {x, y};
      local_map += world_map[tmp_key];
    }

  Cloud_t::Ptr local_map_ptr(new Cloud_t(local_map));
  sensor_msgs::PointCloud2 local_map_msg;
  pcl::toROSMsg(*local_map_ptr, local_map_msg);
  local_map_msg.header.frame_id = "map";
  local_map_pub.publish(local_map_msg);

#ifdef USE_GPU_PCL
  gpu_ndt.setTransformationEpsilon(trans_eps);
  gpu_ndt.setStepSize(step_size);
  gpu_ndt.setResolution(ndt_res);
  gpu_ndt.setMaximumIterations(max_iter);
  gpu_ndt.setInputTarget(local_map_ptr);
#else
  ndt.setTransformationEpsilon(trans_eps);
  ndt.setStepSize(step_size);
  ndt.setResolution(ndt_res);
  ndt.setMaximumIterations(max_iter);
  ndt.setInputTarget(local_map_ptr);
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

#ifdef MY_EXTRACT_SCANPOSE // map_pose.csv
  csv_stream.open(_output_directory + csv_filename);
  csv_stream << "key,sequence,sec,nsec,x,y,z,roll,pitch,yaw" << std::endl;
#endif // MY_EXTRACT_SCANPOSE

  std::cout << "Start processing incoming clouds... " << std::endl;
  ros::spin();
  return 0;
}