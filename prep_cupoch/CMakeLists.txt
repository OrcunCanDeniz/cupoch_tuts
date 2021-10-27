cmake_minimum_required(VERSION 3.15)
project(prep_cupoch)

find_package(catkin REQUIRED COMPONENTS
        roscpp
        sensor_msgs
        # pcl_conversions
        cupoch_conversions
        swri_profiler
        geometry_msgs
        )

find_package(GFlags REQUIRED QUIET)
find_package(CUDA REQUIRED QUIET)
# find_package(PCL REQUIRED QUIET)
find_package(thrust REQUIRED QUIET)

GA_CHECK_CUDA()

set_directory_properties(PROPERTIES COMPILE_DEFINITIONS "")

APPEND_TARGET_ARCH_FLAGS()

set(CUDA_NVCC_FLAGS ${CUDA_NVCC_FLAGS} ${CUPOCH_NVCC_FLAGS})
add_definitions(${CUPOCH_DEFINITIONS})
message(STATUS "zs: ga_points_downsampler CUDA_NVCC_FLAGS = " ${CUDA_NVCC_FLAGS})

###################################
## catkin specific configuration ##
###################################
catkin_package(
        INCLUDE_DIRS include
        LIBRARIES
        CATKIN_DEPENDS
        roscpp
        sensor_msgs
        cupoch_conversions
)


###########
## Build ##
###########
include_directories(
        include
        ${Open3D_INCLUDE_DIRS}
        ${catkin_INCLUDE_DIRS}
        ${CUDA_INCLUDE_DIRS}
        ${CUPOCH_INCLUDE_DIRS}
)
link_directories(${CUPOCH_LIBRARY_DIRS})
link_libraries(${CUPOCH_LIBRARIES})

add_subdirectory(src/concatenate_data/)
add_subdirectory(src/box_filter/)
add_subdirectory(src/prep_pipeline/)

# set(GA_CONFIG_FOLDER ${CMAKE_CURRENT_LIST_DIR}/config)
# set(GA_LOG_FOLDER ${CMAKE_CURRENT_LIST_DIR}/log)
# set(GA_RES_FOLDER ${CMAKE_CURRENT_LIST_DIR}/res)
# file(COPY ${GA_CONFIG_FOLDER} DESTINATION ${CMAKE_RUNTIME_OUTPUT_DIRECTORY})
# file(COPY ${GA_LOG_FOLDER} DESTINATION ${CMAKE_RUNTIME_OUTPUT_DIRECTORY})
# file(COPY ${GA_RES_FOLDER} DESTINATION ${CMAKE_RUNTIME_OUTPUT_DIRECTORY})

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
        FILES_MATCHING PATTERN "*.h"
)

# install(DIRECTORY rviz/
#     DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/rviz
# )

install(DIRECTORY launch/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
