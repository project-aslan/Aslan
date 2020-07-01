cmake_minimum_required(VERSION 2.8.3)
project(pcd_loader)

find_package(aslan_build_flags REQUIRED)

find_package(catkin REQUIRED)
find_package(aslan_msgs REQUIRED)
find_package(CURL REQUIRED)
find_package(catkin REQUIRED COMPONENTS
        roscpp
        std_msgs
        geometry_msgs
        tf
        pcl_ros
        aslan_msgs
        )

set(CMAKE_CXX_FLAGS "-O2 -Wall ${CMAKE_CXX_FLAGS}")

catkin_package()

include_directories(
        include
        ${catkin_INCLUDE_DIRS}
        ${aslan_msgs_INCLUDE_DIRS}
        ${PCL_IO_INCLUDE_DIRS}
        ${CURL_INCLUDE_DIRS}
)

add_library(get_file
        lib/map_file/get_file.cpp
        )
target_link_libraries(get_file ${CURL_LIBRARIES})

add_executable(points_map_loader src/points_map_loader.cpp)
target_link_libraries(points_map_loader ${catkin_LIBRARIES} get_file ${CURL_LIBRARIES} ${PCL_IO_LIBRARIES})
add_dependencies(points_map_loader ${catkin_EXPORTED_TARGETS})

## Install executables and/or libraries
install(TARGETS get_file points_map_loader
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

## Install project namespaced headers
install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
