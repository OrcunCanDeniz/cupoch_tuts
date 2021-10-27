#include <concatenate_data/concatenate_data.hpp>
#include <swri_profiler/profiler.h>

namespace gpuac
{

bool profile = true;

cupochPrep::cupochPrep()
{
  input_topics_ = {"/lidar/back_left/points_raw", "/lidar/back_right/points_raw","/lidar/front_right/points_raw",
                      "/lidar/front_left/points_raw", "/lidar/parent/points_raw"};
  filters_.resize(input_topics_.size());
  
  tfListener_ptr_ = std::make_shared<tf2_ros::TransformListener>(tf_buffer);

  for (int d = 0; d < filters_.size(); d++)
  {
    cupochCloud_map_.insert(std::make_pair((std::string)(input_topics_[d]), nullptr));
    topic2Transform_map.insert(std::make_pair((std::string)(input_topics_[d]), nullptr));

    filters_[d].reset(new ros::Subscriber());
    *filters_[d] = pnh_.subscribe<sensor_msgs::PointCloud2>(
      (std::string)(input_topics_[d]), 10, bind(&cupochPrep::cloudCB, this, _1, (std::string)input_topics_[d]));
    ROS_INFO_STREAM("Subscribed to, " << input_topics_[d]);

  concat_pub = pnh_.advertise<sensor_msgs::PointCloud2>("/cupoch/concat/output", 1);
  }
}


void cupochPrep::cloudCB(const sensor_msgs::PointCloud2ConstPtr& pc_msg, std::string topic_name)
{
  SWRI_PROFILE("cloudCB");
  std::shared_ptr<cupoch::geometry::PointCloud> cbCloud{std::make_shared<cupoch::geometry::PointCloud>()};
  
  if (topic2Transform_map[topic_name] == nullptr) // Transform Handler
  {
    bool is_tf_ok = false;
    geometry_msgs::TransformStamped transformStamped;
    try
    {
      transformStamped = tf_buffer.lookupTransform("lidar/parent/os_sensor", pc_msg->header.frame_id, ros::Time(0));
      is_tf_ok = true;
    }
    catch(tf::TransformException ex)
    {
      ROS_ERROR("Cupoch Concat, %s", ex.what());
    }

    if (is_tf_ok)
    {
      tf2Matrix(transformStamped, topic2Transform_map[topic_name]);
    }
    ROS_INFO_STREAM("Cached transform for, " <<  pc_msg->header.frame_id );
  }
  
  if (cupochCloud_map_[topic_name] == nullptr)
  {
    cupochCloud_map_[topic_name].reset(new cupoch::geometry::PointCloud);
  }
  
  cupoch_conversions::rosToCupoch(pc_msg, cbCloud, true); //lock cbCloud by mutex  
  *cupochCloud_map_[topic_name] = *cbCloud;

  const bool all_pc_ok = std::all_of(
    std::begin(cupochCloud_map_), std::end(cupochCloud_map_),
    [](const auto & e) { return e.second != nullptr; });
  const bool all_tf_ok = std::all_of(
    std::begin(topic2Transform_map), std::end(topic2Transform_map),
    [](const auto & e) { return e.second != nullptr; });
  
  if (all_pc_ok && all_tf_ok) // all point clouds and tfs must be gathered before concatenation
  {
    combineClouds();
  }  

  // auto t2 = ros::WallTime::now();
  // ROS_INFO_STREAM("rosToCupoch processing_time: " << (t2 - t1).toSec() * 1000.0 << "[ms]");
}

void cupochPrep::combineClouds()
{
  for (const auto & e : cupochCloud_map_) 
  {
    if (e.first != "/lidar/parent/points_raw") 
    {
      // concat here
      *transedPC = cupochCloud_map_[e.first]->Transform(*topic2Transform_map[e.first]);
      *cupochCloud_map_["/lidar/parent/points_raw"] += *transedPC;
      cupochCloud_map_[e.first] = nullptr;
    }  
  }

  cupoch_conversions::cupochToRos(cupochCloud_map_["/lidar/parent/points_raw"], m_pub_cupoch_pc, "/lidar/parent/os_sensor");

  m_pub_cupoch_pc.header.stamp = ros::Time::now();
  concat_pub.publish(m_pub_cupoch_pc);
}

void cupochPrep::tf2Matrix(geometry_msgs::TransformStamped transform_stamped,  std::shared_ptr<Eigen::Matrix4f>& mat)
{
  Eigen::Isometry3d isometry = tf2::transformToEigen(transform_stamped);

  if (mat == nullptr)  mat.reset(new Eigen::Matrix4f);
  *mat = isometry.matrix().cast<float>();
}

} //namespace
