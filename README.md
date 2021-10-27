
## Cupoch Installation ##
Download eigen 3.3.7. 

    cd eigen-3.3.7
    mkdir build && cd build
    cmake .. -DCMAKE_INSTALL_PREFIX=/usr
    sudo make install


Follow tutorial below to install cupoch.
https://github.com/ZhenshengLee/perception_cupoch/blob/noetic/cupoch_conversions/docs/tutorial.md

## Interfacing ROS ##
https://github.com/ZhenshengLee/perception_cupoch

Clone repo and build as 
        
    export GPUAC_COMPILE_WITH_CUDA=1
    catkin build -DCMAKE_BUILD_TYPE=Release

## Potential Issues ##
- https://github.com/neka-nat/cupoch/issues/83
- https://github.com/ZhenshengLee/perception_cupoch/issues/14#issuecomment-915913136


### Takeaways from this work ###
- Transform+concat : ~2.2 ms for cupoch, ~3.3ms for pcl. 
- " Global crop -> self crop -> voxel grid " pipeline in single node: pcl ~21 ms cupoch ~12ms
- **Some pointcloud registration algorithms are implemented in cupoch. Might be useful in the future**