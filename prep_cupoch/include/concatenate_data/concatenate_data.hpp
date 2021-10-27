#pragma once 

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <Eigen/Eigen>
#include "tf2_eigen/tf2_eigen.h"
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/convert.h>
#include <tf2/transform_datatypes.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include <tf/transform_listener.h>
#include "cupoch_conversions/cupoch_conversions.h"
#include "tf/transform_datatypes.h"
#include "Eigen/Core"
#include "Eigen/Geometry"
#include <tf_conversions/tf_eigen.h>

namespace gpuac
{
class cupochPrep
{
    public:
        cupochPrep();

    private:
        void cloudCB (const sensor_msgs::PointCloud2ConstPtr& pc_msg,  std::string topic_name);
        void tf2Matrix(geometry_msgs::TransformStamped transform_stamped, std::shared_ptr<Eigen::Matrix4f>& mat);
        void combineClouds();

        ros::NodeHandle pnh_;
        ros::NodeHandle nh;
        ros::Publisher concat_pub;
        
        tf::TransformListener listener;
        std::vector<std::string> input_topics_;
        std::vector<boost::shared_ptr<ros::Subscriber>> filters_;

        std::shared_ptr<cupoch::geometry::PointCloud> transedPC{std::make_shared<cupoch::geometry::PointCloud>()};

        sensor_msgs::PointCloud2 m_pub_cupoch_pc;

        std::map<std::string, std::shared_ptr<cupoch::geometry::PointCloud>> cupochCloud_map_;
        std::map<std::string, sensor_msgs::PointCloud2::ConstPtr> cloud_stdmap_tmp_;
        std::map<std::string, std::shared_ptr<Eigen::Matrix4f>> topic2Transform_map;

        tf2_ros::Buffer tf_buffer;
        std::shared_ptr<tf2_ros::TransformListener> tfListener_ptr_;
}; //class
} //namespace