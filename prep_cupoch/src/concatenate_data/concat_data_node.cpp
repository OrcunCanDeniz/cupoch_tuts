#include <concatenate_data/concatenate_data.hpp>


int main (int argc, char** argv)
{
  // Initialize ROS
    ros::init (argc, argv, "concat_node");
    cupoch::utility::InitializeAllocator(cupoch::utility::PoolAllocation, 64000000); // 2nd argument is memory to be allocated in bytes

    gpuac::cupochPrep app;

    // Spin
    ros::spin ();
}