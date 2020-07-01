cmake_minimum_required(VERSION 2.8.3)
project(umrr_driver)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  dynamic_reconfigure
  rospy
  std_msgs
  message_generation
)




catkin_python_setup()

generate_dynamic_reconfigure_options(
  cfg/pc2filter.cfg
)
## Generate services in the 'srv' folder
 add_service_files(
   FILES
   sens_status.srv
   sens_param.srv
   sens_param_req.srv
 )

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs )

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES umrr_drivers
#  CATKIN_DEPENDS rospy message_runtime
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)
