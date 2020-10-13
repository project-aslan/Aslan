cmake_minimum_required(VERSION 2.8.3)
project(sd_vehicle_interface)

find_package(catkin REQUIRED COMPONENTS
        roscpp
        rosconsole
        std_msgs
        can_msgs
        socketcan_interface
	sensor_msgs
	aslan_msgs
        )

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -w")

catkin_package(
  INCLUDE_DIRS
    include
  LIBRARIES
    socketcan_to_topic
    topic_to_socketcan
  CATKIN_DEPENDS roscpp std_msgs can_msgs aslan_msgs socketcan_interface rosconsole
)


include_directories(
        include
        src/sd_vehicle_interface/
        ${catkin_INCLUDE_DIRS}
)

# socketcan_to_topic
add_library(socketcan_to_topic
        src/socketcan_bridge/socketcan_to_topic.cpp
        )
target_link_libraries(socketcan_to_topic
        ${catkin_LIBRARIES}
        )
add_dependencies(socketcan_to_topic
        ${catkin_EXPORTED_TARGETS}
        )

# topic_to_socketcan
add_library(topic_to_socketcan
        src/socketcan_bridge/topic_to_socketcan.cpp
        )
target_link_libraries(topic_to_socketcan
        ${catkin_LIBRARIES}
        )
add_dependencies(topic_to_socketcan
        ${catkin_EXPORTED_TARGETS}
        )

# topic_to_socketcan_node
add_executable(socketcan_bridge_node
  src/socketcan_bridge/socketcan_bridge_node.cpp

)
target_link_libraries(socketcan_bridge_node
        socketcan_to_topic
        topic_to_socketcan
  ${catkin_LIBRARIES}
)


add_executable(sd_vehicle_interface_node src/sd_vehicle_interface/sd_vehicle_interface_main.cpp
    src/sd_vehicle_interface/sd_vehicle_interface.cpp
		src/sd_vehicle_interface/sd_control.cpp
		src/sd_vehicle_interface/sd_gps_imu.cpp)

add_dependencies(sd_vehicle_interface_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(sd_vehicle_interface_node ${catkin_LIBRARIES})
target_link_libraries(sd_vehicle_interface_node ${catkin_LIBRARIES} ${CMAKE_CURRENT_SOURCE_DIR}/lib/libsd_interface_lib.so)



install(
  TARGETS
        sd_vehicle_interface_node
        socketcan_to_topic
        topic_to_socketcan
        socketcan_bridge_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  DIRECTORY
    include/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
install(FILES config.sh
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})