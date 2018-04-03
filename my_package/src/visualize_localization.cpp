/*
 *  Copyright (c) 2015, Nagoya University
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 *
 *  * Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 *  * Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 *  * Neither the name of Autoware nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 *  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 *  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *  DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 *  FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 *  DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 *  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 *  OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include <ros/ros.h>
#include <ros/time.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>


#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>

#include <chrono>
#include <queue>

#define MAX_MEASUREMENT_RANGE 200.0

typedef pcl::PointXYZI Point_t;
typedef pcl::PointCloud<Point_t> Cloud_t;

static ros::Publisher cloud_pub;
static std::queue<sensor_msgs::PointCloud2> cloud_msg_queue;
static ros::Time cloud_time;
static double measurement_range = MAX_MEASUREMENT_RANGE;

static void cloud_callback(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg)
{
  cloud_msg_queue.push(*cloud_msg);
  return;  
}

static void pose_callback(const geometry_msgs::PoseStamped::ConstPtr& pose_msg)
{
  // if(cloud_msg_queue.size() == 0)
  // {
  //   ROS_WARN("Empty cloud_msg_vector");
  //   return;
  // }

  // sensor_msgs::PointCloud2 cloud_msg = cloud_msg_queue.front();
  // cloud_msg_queue.pop();
  // if(pose_msg->header.stamp != cloud_msg.header.stamp)
  // {
  //   ROS_WARN("Different pose and cloud timestamps!");
  //   return;
  // }

  // Cloud_t cloud;
  // pcl::fromROSMsg(cloud_msg, cloud);

  double x_ = pose_msg->pose.position.x;
  double y_ = pose_msg->pose.position.y;
  double z_ = pose_msg->pose.position.z;

  tf::Quaternion q(pose_msg->pose.orientation.x, pose_msg->pose.orientation.y, pose_msg->pose.orientation.z, pose_msg->pose.orientation.w);
  // q.normalize();
  double roll_, pitch_, yaw_;
  tf::Matrix3x3(q).getEulerYPR(roll_, pitch_, yaw_);

  // Eigen::Affine3d pose_transform;
  // pcl::getTransformation(x_, y_, z_, roll_, pitch_, yaw_, pose_transform);
  // pcl::transformPointCloud(cloud, cloud, pose_transform);

  // pcl::toROSMsg(cloud, cloud_msg);
  // cloud_msg.header.frame_id = "map";
  // cloud_pub.publish(cloud_msg);

  tf::TransformBroadcaster br;
  tf::Transform transform;
  transform.setOrigin(tf::Vector3(pose_msg->pose.position.x, pose_msg->pose.position.y, pose_msg->pose.position.z));
  transform.setRotation(q);
  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "map", "velodyne"));
  ROS_INFO("Broadcasting transform...");
   
  return;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "visualize_localization");

  ros::NodeHandle nh;
  // Publishers
  cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("/points_localized", 10);

  // Subscribers
  // ros::Subscriber cloud_sub = nh.subscribe("/points_raw", 1000, cloud_callback);
  ros::Subscriber pose_sub = nh.subscribe("/localizer_pose", 1000, pose_callback);
  ROS_INFO("Waiting for clouds and pose...");
  ros::spin();
  return 0;
}
