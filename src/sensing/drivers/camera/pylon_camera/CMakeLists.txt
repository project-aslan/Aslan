cmake_minimum_required(VERSION 2.8.3)
project(pylon_camera)

add_definitions("-std=gnu++11")
#set(CMAKE_CXX_FLAGS "-g -Wall -Wno-unknown-pragmas -Wno-delete-non-virtual-dtor -Wno-unused-variable")
set(
    CATKIN_COMPONENTS
     actionlib
     camera_control_msgs
     camera_info_manager
     cv_bridge
     diagnostic_updater
     image_geometry
     image_transport
     roscpp
     roslaunch
     sensor_msgs
)

find_package(Pylon QUIET)
if (NOT ${Pylon_FOUND})
    include("${CMAKE_CURRENT_SOURCE_DIR}/cmake/FindPylon.cmake")
endif()
find_package(
    catkin REQUIRED
    COMPONENTS
     ${CATKIN_COMPONENTS}
     #std_srvs
     roslint
)

catkin_package(
    INCLUDE_DIRS
     include
    LIBRARIES
     ${PROJECT_NAME}
    CATKIN_DEPENDS
     ${CATKIN_COMPONENTS}
)

set(
    ROSLINT_CPP_OPTS
     "--extensions=cpp,h,hpp" "--filter=-runtime/references,-readability/todo,-build/include_what_you_use"
)

roslint_cpp(
    src/${PROJECT_NAME}/binary_exposure_search.cpp
    src/${PROJECT_NAME}/encoding_conversions.cpp
    src/${PROJECT_NAME}/main.cpp
    src/${PROJECT_NAME}/${PROJECT_NAME}_node.cpp
    src/${PROJECT_NAME}/${PROJECT_NAME}_parameter.cpp
    src/${PROJECT_NAME}/${PROJECT_NAME}.cpp
    src/${PROJECT_NAME}/write_device_user_id_to_camera.cpp
    include/${PROJECT_NAME}/binary_exposure_search.h
    include/${PROJECT_NAME}/encoding_conversions.h
    include/${PROJECT_NAME}/${PROJECT_NAME}_node.h
    include/${PROJECT_NAME}/${PROJECT_NAME}_parameter.h
    include/${PROJECT_NAME}/${PROJECT_NAME}.h
    include/${PROJECT_NAME}/internal/${PROJECT_NAME}.h
    include/${PROJECT_NAME}/internal/impl/${PROJECT_NAME}_base.hpp
    include/${PROJECT_NAME}/internal/impl/${PROJECT_NAME}_dart.hpp
    include/${PROJECT_NAME}/internal/impl/${PROJECT_NAME}_gige.hpp
    include/${PROJECT_NAME}/internal/impl/${PROJECT_NAME}_usb.hpp
)

roslaunch_add_file_check(launch)

include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${catkin_INCLUDE_DIRS}
    ${Pylon_INCLUDE_DIRS}
)

# Add library
add_library(
    ${PROJECT_NAME}
     src/${PROJECT_NAME}/binary_exposure_search.cpp
     src/${PROJECT_NAME}/encoding_conversions.cpp
     src/${PROJECT_NAME}/${PROJECT_NAME}.cpp
     src/${PROJECT_NAME}/${PROJECT_NAME}_node.cpp
     src/${PROJECT_NAME}/${PROJECT_NAME}_parameter.cpp
)

target_link_libraries(
    ${PROJECT_NAME}
     ${catkin_LIBRARIES}
     ${Pylon_LIBRARIES}
)

add_dependencies(
    ${PROJECT_NAME}
     ${catkin_EXPORTED_TARGETS}
)

# Add pylon_camera_node
add_executable(
    ${PROJECT_NAME}_node
     src/${PROJECT_NAME}/main.cpp
)

target_link_libraries(
    ${PROJECT_NAME}_node
     ${PROJECT_NAME}
)

add_executable(
    write_device_user_id_to_camera
     src/${PROJECT_NAME}/write_device_user_id_to_camera.cpp
)

target_link_libraries(
    write_device_user_id_to_camera
     ${Pylon_LIBRARIES}
)

add_dependencies(
    ${PROJECT_NAME}_node
     ${catkin_EXPORTED_TARGETS}
)

catkin_python_setup()

install(
    DIRECTORY
     launch/
    DESTINATION
     ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
    FILES_MATCHING PATTERN "*.launch"
)

install(
    DIRECTORY
     config/
    DESTINATION
     ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
    FILES_MATCHING PATTERN "*.yaml"
)

install(
    PROGRAMS
     scripts/file_sequencer.py
     scripts/grab_and_save_image_action_server.py
     scripts/result_bag_to_action.py
     scripts/sequence_to_file.py
     scripts/toggle_camera
    DESTINATION
     ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(
    TARGETS
     ${PROJECT_NAME}
     ${PROJECT_NAME}_node
     write_device_user_id_to_camera
    LIBRARY DESTINATION
     ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION
     ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
    DIRECTORY
     include/${PROJECT_NAME}/
    DESTINATION
     ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
    FILES_MATCHING PATTERN
     "*.h"
    PATTERN "internal" EXCLUDE
)

## Testing ##
# All Jenkins-Tests are now in the pylon_camera_tests-pkg
############

###############
## QtCreator ##
###############
# entry for QtCreator to show all files
file(GLOB children ${CMAKE_CURRENT_SOURCE_DIR}/*)
foreach(child ${children})
  if(IS_DIRECTORY ${child})
    file(GLOB_RECURSE dir_files "${child}/*")
    list(APPEND ${PROJECT_NAME}_extra_files ${dir_files})
  endif()
endforeach()
add_custom_target(dummy_${PROJECT_NAME} SOURCES ${${PROJECT_NAME}_extra_files})

