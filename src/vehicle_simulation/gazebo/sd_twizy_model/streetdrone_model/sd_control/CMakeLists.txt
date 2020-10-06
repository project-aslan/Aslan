cmake_minimum_required(VERSION 2.8.3)
project(sd_control)

set (CMAKE_CXX_STANDARD 11)

find_package(catkin REQUIRED COMPONENTS
  gazebo_ros
  roscpp
  tf
  nav_msgs
  aslan_msgs
  rospy

)

catkin_package(CATKIN_DEPENDS rospy)

find_package(gazebo 9 REQUIRED)

install(DIRECTORY launch/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
        FILES_MATCHING PATTERN "*.launch" PATTERN "*.machine" PATTERN "*.yaml" PATTERN "*.urdf"
        )

install(DIRECTORY config/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
        FILES_MATCHING PATTERN "*.yaml"
        )

catkin_install_python(PROGRAMS
        src/sd_teleop_keyboard.py
        DESTINATION share/${PROJECT_NAME})

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
  include/
  )

add_library(SdControlPlugin SHARED src/sd_control_plugin.cpp)
target_link_libraries(SdControlPlugin ${GAZEBO_LIBRARIES})
add_dependencies(SdControlPlugin ${catkin_EXPORTED_TARGETS})

add_executable(sd_teleop_joy src/sd_teleop_joy.cpp src/sd_teleop_joy_node.cpp)
add_dependencies(sd_teleop_joy ${catkin_EXPORTED_TARGETS})

target_link_libraries(sd_teleop_joy
  ${catkin_LIBRARIES})

install(TARGETS SdControlPlugin sd_teleop_joy
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

