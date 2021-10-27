#include <box_filter/box_filter.hpp>


int main (int argc, char** argv)
{
  // Initialize ROS
    ros::init (argc, argv, "cupoch_box_filter_node");
    cupoch::utility::InitializeAllocator(cupoch::utility::PoolAllocation, 512000000); // 2nd argument is memory to be allocated in bytes

    gpuac::cupochPrep app;


    // Spin
    ros::spin ();
}