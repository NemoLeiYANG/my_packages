#include <iostream>
#include <string>
#include <math.h>
#include <unistd.h>
#include <unordered_map>
#include <chrono>

#include <pcl/common/common.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/statistical_outlier_removal.h>

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
		std::cout << "Usage: rosrun my_package map_downsampler MAP_FILE.pcd [MAP_FILE2.pcd [MAP_FILE3.pcd ..]]";
		return -1;
	}
	std::chrono::time_point<std::chrono::system_clock> time_start = std::chrono::system_clock::now();

	Cloud_t src; // all input map
	for(int i = 1; i < argc; i++)
	{	
		Cloud_t tmp;
		std::string inputFile = argv[i];

	  std::cout << "Loading " << inputFile << std::endl;
		if(pcl::io::loadPCDFile<Point_t>(inputFile, tmp) == -1)
	  {
	    std::cerr << "Couldn't read " << inputFile << "." << std::endl;
	    return -1;
	  }
	  else
	  	src += tmp;
	}
  std::cout << src.size() << " data points loaded." << std::endl;
	Cloud_t::Ptr src_ptr(new Cloud_t(src));
	Cloud_t::Ptr filtered_ptr(new Cloud_t());

	// Create the filtering object
  pcl::StatisticalOutlierRemoval<Point_t> sor;
  sor.setInputCloud(src_ptr);
  sor.setMeanK(1000);
  sor.setStddevMulThresh(1.0);
  sor.filter(*filtered_ptr);

	std::cout << filtered_ptr->size() << " points after filtering." << std::endl;
	pcl::io::savePCDFileBinary("inliers.pcd", *filtered_ptr);
	
	double time_diff = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::system_clock::now() - time_start).count() / 1000.0;
	std::cout << "Finished. Saved pointcloud to inliers.pcd. Took " << time_diff << " ms." << std::endl;
	return 0;
}