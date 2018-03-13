#include <iostream>
#include <string>
#include <math.h>
#include <unistd.h>
#include <unordered_map>
#include <chrono>
#include <mutex>
#include <omp.h>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
// #include <velodyne_pointcloud/point_types.h>

#include <pcl/common/common.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
// #include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>

#define TILE_WIDTH 5
typedef pcl::PointXYZI Point_t;
typedef pcl::PointCloud<Point_t> Cloud_t;

struct Key
{
	int x;
	int y;
};

bool operator==(const Key& lhs, const Key& rhs) {
    return lhs.x == rhs.x && lhs.y == rhs.y;
}

namespace std
{
  template <>
  struct hash<Key>
  {
    std::size_t operator()(const Key& k) const
    {
      return hash<int>()(k.x) ^ (hash<int>()(k.y) << 1);
    }
  };
}

int main(int argc, char** argv)
{
	// Get input pointcloud first
	if(argc < 2)
	{
		std::cerr << "Missing input .pcd file." << std::endl;
		std::cout << "Usage: rosrun my_package map_downsampler VOXEL_LEAF_SIZE MAP_FILE.pcd [MAP_FILE2.pcd [MAP_FILE3.pcd ..]] OUTPUT_FILE.pcd" << std::endl;
		return -1;
	}

	double voxel_leaf_size = std::stod(argv[1]);
	std::cout << "Using voxel_leaf_size = " << voxel_leaf_size << std::endl;

	pcl::PointCloud<pcl::PointXYZI> src; // all input map
	for(int i = 2; i < argc - 1; i++)
	{	
		pcl::PointCloud<pcl::PointXYZI> tmp;
		std::string inputFile = argv[i];

	  std::cout << "Loading " << inputFile << std::endl;
		if(pcl::io::loadPCDFile<pcl::PointXYZI>(inputFile, tmp) == -1)
	  {
	    std::cerr << "Couldn't read " << inputFile << "." << std::endl;
	    return -1;
	  }
	  else
	  	src += tmp;
	}
  std::cout << src.size() << " data points loaded." << std::endl;

  std::unordered_map<Key, pcl::PointCloud<pcl::PointXYZI>> worldMap;

  // Allocate each point in source map to worldMap
  std::chrono::time_point<std::chrono::system_clock> time_start;
  time_start = std::chrono::system_clock::now();

  for(pcl::PointCloud<pcl::PointXYZI>::const_iterator item = src.begin(); item < src.end(); item++)
  {
  	// Get 2D point
  	Key key;
  	key.x = int(floor(item->x / TILE_WIDTH));
  	key.y = int(floor(item->y / TILE_WIDTH));

	  worldMap[key].push_back(*item);
	}

	Cloud_t downsampledWorldMap;
 // #pragma omp parallel 
 // {
 //   #pragma omp single
 //   {
	  for(auto x = worldMap.begin(); x != worldMap.end(); x++) 
     // #pragma omp task
	    {
				// Downsample the source cloud, apply voxelgrid filter
				Cloud_t::Ptr src_ptr(new Cloud_t(x->second));
				Cloud_t downsampledLocalMap;
				pcl::VoxelGrid<Point_t> voxel_grid_filter;
				voxel_grid_filter.setLeafSize(voxel_leaf_size, voxel_leaf_size, voxel_leaf_size);
				voxel_grid_filter.setInputCloud(src_ptr);
				voxel_grid_filter.filter(downsampledLocalMap);

				downsampledWorldMap += downsampledLocalMap;
      }
   // } 
 // }
	std::cout << downsampledWorldMap.size() << " points after downsampling." << std::endl;
	pcl::io::savePCDFileBinary(argv[argc-1], downsampledWorldMap);
	
	double time_diff = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::system_clock::now() - time_start).count() / 1000.0;
	std::cout << "Finished. Saved pointcloud to " << argv[argc-1] << ". Took " << time_diff << " ms." << std::endl;
	return 0;
}