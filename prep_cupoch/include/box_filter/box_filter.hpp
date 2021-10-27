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
        void cloudCB (const sensor_msgs::PointCloud2ConstPtr& pc_msg);

        ros::NodeHandle pnh_;
        ros::NodeHandle nh;

        ros::Subscriber pointcloud_sub;
        ros::Publisher crop_pub;

        sensor_msgs::PointCloud2 m_pub_cupoch_pc;

        Eigen::Matrix<float, 1, 3> min_bound, max_bound; 
        cupoch::geometry::AxisAlignedBoundingBox<3> filter_box;

        std::shared_ptr<cupoch::geometry::PointCloud> cupochCloud{std::make_shared<cupoch::geometry::PointCloud>()};
        std::shared_ptr<cupoch::geometry::PointCloud> cropCloud{std::make_shared<cupoch::geometry::PointCloud>()};


}; //class
} //namespace