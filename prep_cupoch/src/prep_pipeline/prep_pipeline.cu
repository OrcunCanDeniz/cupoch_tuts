#include <prep_pipeline/prep_pipeline.hpp>
#include <swri_profiler/profiler.h>

namespace gpuac
{

cupochPrep::cupochPrep()
{
  global_min_bound << -30.0, -30.0, -5.0;
  global_max_bound << 50.0, 30.0, 3.5;

  self_min_bound << -1.7, -1.5, 0.0;
  self_max_bound << 7.0, 1.5, 3.5;
  
  global_filter_box = cupoch::geometry::AxisAlignedBoundingBox<3>(global_min_bound, global_max_bound);
  self_filter_box = cupoch::geometry::AxisAlignedBoundingBox<3>(self_min_bound, self_max_bound);

  pointcloud_sub = pnh_.subscribe<sensor_msgs::PointCloud2>("/lidar/concatenated/pointcloud/", 10, &cupochPrep::cloudCB, this);
  crop_pub = pnh_.advertise<sensor_msgs::PointCloud2>("/cupoch/prep_pipeline/output", 1);
  ROS_INFO_STREAM("Box filter initialized. ");
}

void cupochPrep::cloudCB(const sensor_msgs::PointCloud2ConstPtr& pc_msg)
{
  SWRI_PROFILE("cropandVoxel");
  cupoch_conversions::rosToCupoch(pc_msg, cupochCloud, true);

  cupochCloud = cupochCloud->Crop(global_filter_box, false);
  cupochCloud = cupochCloud->Crop(self_filter_box, true); // true makes func to return points out of filter box. custom implementation

  cupochCloud = cupochCloud->VoxelDownSample(0.3);

  cupoch_conversions::cupochToRos(cupochCloud, m_pub_cupoch_pc, "base_link");

  m_pub_cupoch_pc.header.stamp = ros::Time::now();
  crop_pub.publish(m_pub_cupoch_pc);
}

} //namespace
