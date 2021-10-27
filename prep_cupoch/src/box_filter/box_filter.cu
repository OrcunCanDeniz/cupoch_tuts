#include <box_filter/box_filter.hpp>
#include <swri_profiler/profiler.h>

namespace gpuac
{

cupochPrep::cupochPrep()
{

  min_bound << -10, -10, -5;
  max_bound << 10, 10, 5;

  filter_box = cupoch::geometry::AxisAlignedBoundingBox<3>(min_bound, max_bound);

  pointcloud_sub = pnh_.subscribe<sensor_msgs::PointCloud2>("/lidar/concatenated/pointcloud/", 10, &cupochPrep::cloudCB, this);
  crop_pub = pnh_.advertise<sensor_msgs::PointCloud2>("/cupoch/crop_box/output", 1);
  ROS_INFO_STREAM("Box filter initialized. ");
}


void cupochPrep::cloudCB(const sensor_msgs::PointCloud2ConstPtr& pc_msg)
{
  SWRI_PROFILE("boxFilterCB");

  cupoch_conversions::rosToCupoch(pc_msg, cupochCloud, true);

  cropCloud = cupochCloud->Crop(filter_box, true); // second argument chooses whether fillter box will be negatived or not. Custom implemented feature on crop function. 
  
  cupoch_conversions::cupochToRos(cropCloud, m_pub_cupoch_pc, "/lidar/parent/os_sensor");

  m_pub_cupoch_pc.header.stamp = ros::Time::now();
  crop_pub.publish(m_pub_cupoch_pc);

  // auto t2 = ros::WallTime::now();
  // ROS_INFO_STREAM("rosToCupoch processing_time: " << (t2 - t1).toSec() * 1000.0 << "[ms]");
}

} //namespace
