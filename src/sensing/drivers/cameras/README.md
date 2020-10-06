### Camera Drivers

This directory contains all the drivers for the cameras, Project ASLAN is supporting. Most drivers require additional packages to be installed separately, otherwise the build will fail. The CMAKE is skipping the build on these packages.

#### Build
In order to build any camera driver, delete the file: "CATKIN_IGNORE" from its directory. Then follow the instructions as described on the READMEs.

##### 1. pointgrey_camera_driver
First install the camera driver.
```
sudo apt-get install ros-melodic-pointgrey-camera-driver
```
To build this package individually:
```
catkin build pointgrey_camera_driver
OR
catkin_make --only-pkg-with-deps pointgrey_camera_driver #if you have previously built your workspace with catkin_make
```

To run this camera package in the terminal use the following command:

```
roslaunch pointgrey_camera_driver camera.launch
```
For more details follow the instructions [here](http://wiki.ros.org/pointgrey_camera_driver)

##### 2. pylon-ros-camera
ROS Driver for Basler cameras.  
Please follow instructions at /pylon_camera/README in this directory.   
For more information visit [here](http://wiki.ros.org/pylon_camera)

##### 3. spinnaker_sdk_camera_driver
Spinnaker SDK ROS Driver.  
Please follow instructions at /spinnaker_sdk_camera_driver/README in this directory.   
For more information visit [here](https://wiki.ros.org/spinnaker_sdk_camera_driver)

##### 4. zed-ros-wrapper
ROS Driver for the ZED Stereo cameras.  
Please follow the instructions at /zed-ros-wrapper/README in this directory.   
For more information visit [here](http://wiki.ros.org/zed-ros-wrapper)