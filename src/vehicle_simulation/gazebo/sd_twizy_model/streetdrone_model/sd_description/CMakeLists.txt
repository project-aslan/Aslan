cmake_minimum_required(VERSION 2.8.3)
project(sd_description)

find_package(catkin REQUIRED COMPONENTS
  urdf
)

find_package(xacro REQUIRED)

catkin_package()

include_directories(
        ${catkin_INCLUDE_DIRS}
)

install(
        DIRECTORY meshes urdf
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
