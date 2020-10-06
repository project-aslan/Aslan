Vehicle interface
======

The StreetDrone Vehicle Interface is the bridge between ROS and the StreetDrone embedded system (XCU) CAN input.  
Control loop feedback mechanisms are also implemented in the interface for vehicle speed and steer control. 

Features
------
1. Continuous software to Drive-by-Wire (DBW) system handshake verification
2. High level safety monitoring with fault response from the supervisor package output
3. Tunable PID and feedforward speed control
4. Tunable yaw to steering percentage conversion map
5. Integration of GPS and IMU vehicle pose information
6. StreetDrone XCU interface for the CAN Bus of both StreetDrone Twizy and StreetDrone ENV200

Disclaimer
------
A trained safety driver must always be present in the vehicle in order to provide critical redundancy when operating the StreetDrone vehicles. 
Please follow the safety instructions provided in the documentation you received with your vehicles.

Requirements
------
##### - Ubuntu 18.04 LTS
##### - ROS Melodic [ros-melodic-desktop-full](http://wiki.ros.org/melodic/Installation/Ubuntu)
##### - Catkin Command Line Tools [catkin_tools](https://catkin-tools.readthedocs.io/en/latest/installing.html)

Architecture
------
*sd_vehicle_interface:* This node is responsible for the main functionality of the package. Here the autonomous to manual mode handshake verification with the vehicle is implemented. 
	The output of the `/sd_diagnostics` topic is used to implement a fault response if an issue is detected in the software.

*sd_control:* A tunable PID and feedforward control function for the StreetDrone Twizy and ENV200. A function for mapping angular velocity request from the software to the vehicle's steering angle is also implemented. 

*sd_gps_imu:* The CAN parsing functions for an OXTS GPS and PEAK CAN GPS-IMU. 

*sd_lib:* The implentation of the StreetDrone CAN protocol.

*sd_typdefs:* Variables and constants definitions.

*socketcan_bridge:* This node comes from the default ROS package socketcan_bridge. The package provides functionality to expose CAN frames from SocketCAN to a ROS Topic. Internally it uses the socketcan_interface from the ros_canopen package, as such, it is capable of dealing with both normal and extended CAN frames.



Building
------

1. Open terminal and clone the entire repo with `git clone`. If you only want this package, go to the `/src` folder of your catkin workspace and copy the contents of only this package.

2. Install the ROS packages: [socketcan_interface](http://wiki.ros.org/socketcan_interface) and [can_msgs](http://wiki.ros.org/can_msgs)
```
sudo apt-get install ros-melodic-socketcan-interface ros-melodic-can-msgs
```

3. Build the package
```
# from the root of your workspace build only this package
catkin build sd_vehicle_interface
```

If you previously built your workspace with `catkin_make`, do `catkin_make --only-pkg-with-deps sd_vehicle_interface`.    


Launch File Parameters
------
sd_vehicle: The vehicle under control. Either "env200" or "twizy" default is "env200".  
sd_gps_imu: The GPS-IMU used. either "oxts" or "peak". Default is oxts.


Launching outside of GUI
------
Before launching, ensure that the CAN interface has been initialised.  
For PEAK CAN USB interfaces, the steps to initialise CAN as can0, are:
```
sudo modprobe peak_usb
sudo ip link set can0 up type can bitrate 500000
```
If not connected to the car, virtual CAN Bus can be used. The steps to initialise virtual CAN as vcan, are:
```
sudo modprobe vcan
sudo ip link add dev can0 type vcan
```
After CAN is initialised, run the node using the following command:
```
source devel/setup.bash
roslaunch sd_vehicle_interface sd_vehicle_interface.launch sd_vehicle:=env200 sd_gps_imu:=oxts
# adjust the launch parameters to your vehicle setup, as previously described
```

