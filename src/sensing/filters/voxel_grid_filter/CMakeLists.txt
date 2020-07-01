cmake_minimum_required(VERSION 2.8.3)
project(voxel_grid_filter)

find_package(aslan_build_flags REQUIRED)

find_package(aslan_msgs REQUIRED)
find_package(catkin REQUIRED COMPONENTS
        roscpp
        pcl_ros
        sensor_msgs
        pcl_conversions
        message_generation
        )

add_message_files(
        FILES
        PointsDownsamplerInfo.msg
)

generate_messages(
        DEPENDENCIES
        std_msgs
)

catkin_package(
        CATKIN_DEPENDS
        roscpp
        pcl_ros
        sensor_msgs
        pcl_conversions
        message_generation
        aslan_msgs
)

###########
## Build ##
###########

include_directories(include ${catkin_INCLUDE_DIRS}
 ${aslan_msgs_INCLUDE_DIRS})
SET(CMAKE_CXX_FLAGS "-O2 -g -Wall ${CMAKE_CXX_FLAGS}")

add_executable(voxel_grid_filter src/voxel_grid_filter.cpp)
add_dependencies(voxel_grid_filter ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(voxel_grid_filter ${catkin_LIBRARIES})
