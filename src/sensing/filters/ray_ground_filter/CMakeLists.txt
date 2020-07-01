cmake_minimum_required(VERSION 2.8.3)
project(ray_ground_filter)

find_package(aslan_build_flags REQUIRED)

find_package(catkin REQUIRED)
find_package(aslan_msgs REQUIRED)
find_package(catkin REQUIRED COMPONENTS
        roscpp
        std_msgs
        sensor_msgs
        pcl_ros
        pcl_conversions
        cv_bridge
        velodyne_pointcloud
        tf
        )

catkin_package(CATKIN_DEPENDS
        roscpp
        std_msgs
        sensor_msgs
        pcl_ros
        pcl_conversions
        cv_bridge
        velodyne_pointcloud
        aslan_msgs
        tf
        )

find_package(Qt5Core REQUIRED)
find_package(OpenCV REQUIRED)
find_package(PCL 1.7 REQUIRED)
find_package(OpenMP)

#[[
# Resolve system dependency on yaml-cpp, which apparently does not
# provide a CMake find_package() module.
find_package(PkgConfig REQUIRED)
pkg_check_modules(YAML_CPP REQUIRED yaml-cpp)
find_path(YAML_CPP_INCLUDE_DIR NAMES yaml_cpp.h PATHS ${YAML_CPP_INCLUDE_DIRS})
find_library(YAML_CPP_LIBRARY NAMES YAML_CPP PATHS ${YAML_CPP_LIBRARY_DIRS})
link_directories(${YAML_CPP_LIBRARY_DIRS})
]]

###########
## Build ##
###########

include_directories(
        ${catkin_INCLUDE_DIRS}
        ${aslan_msgs_INCLUDE_DIRS}
        include
)

SET(CMAKE_CXX_FLAGS "-O2 -g -Wall ${CMAKE_CXX_FLAGS}")

link_directories(${PCL_LIBRARY_DIRS})

add_library(ray_ground_filter_lib SHARED
        src/ray_ground_filter.cpp)

if (OPENMP_FOUND)
    set_target_properties(ray_ground_filter_lib PROPERTIES
            COMPILE_FLAGS ${OpenMP_CXX_FLAGS}
            LINK_FLAGS ${OpenMP_CXX_FLAGS}
            )
endif ()

target_include_directories(ray_ground_filter_lib PRIVATE
        ${OpenCV_INCLUDE_DIRS}
        ${PCL_INCLUDE_DIRS}
        nodes/ray_ground_filter/include
        )

target_link_libraries(ray_ground_filter_lib
        ${catkin_LIBRARIES}
        ${PCL_LIBRARIES}
        ${OpenCV_LIBRARIES}
        ${Qt5Core_LIBRARIES}
        )

add_executable(ray_ground_filter
        src/ray_ground_filter_main.cpp
        )

target_include_directories(ray_ground_filter PRIVATE
        include)

target_link_libraries(ray_ground_filter
        ray_ground_filter_lib)

add_dependencies(ray_ground_filter ${catkin_EXPORTED_TARGETS})

install(TARGETS ray_ground_filter
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        )

install(DIRECTORY include/
        DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}
        PATTERN ".svn" EXCLUDE
        )

