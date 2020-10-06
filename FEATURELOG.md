## Aslan Feature Log

### aslan_docker: 
Aslan docker container linking to projaslan/aslan in [Dockerhub](https://hub.docker.com/r/projaslan/aslan)  

### aslan_tools:
#### aslan_build_flags:
Default build flags when building Aslan

#### aslan_gui: 
A Graphical User Interface (GUI) aiming to facilitate the user when using Aslan.    
**Features:** 
* ROS Nodes launching 
* Launch parameters configuration 
* ROS visualization tools (rqt, rviz): data visualization, rosbag manipulation tools 
* Gazebo simulation launching 
* Built-in documentation.    
Tools used: wxGlade and wxPython. The package runtime_manager from Autoware.ai has been a great inspiration when developing this package.  
  
#### rviz:
Default rviz configuration file for data visualization in Aslan  

### supervisor
#### check_messages_alive:
Developed by StreetDrone and donated to Project Aslan    
**Features:** 
* High level supervisor node, monitoring the health of the system and publishing diagnostic messages for each package. 
* Tracking the status of some topics and publishing an array of the status of each topic in topic /sd_diagnostics using a diagnostics array message.   
    The status of the topic is one of the following:  
    1. ALIVE AND PUBLISHING   
    2. DIDN'T START   
    3. PUBLISHED WRONG DATA   
    4. ERROR   
For each topic a diagnostic status is created and checked according to criteria specified [here](https://github.com/project-aslan/Aslan/tree/master/src/supervisor/check_message_alive).                                            

### vehicle_configuration
Developed by StreetDrone and donated to Project Aslan     
**Features:** 
* Vehicle models visualization in rviz using 3D CAD models. Supports Nissan ENV200 and Renault Twizy	   
* Vehicle description parameters. Supports Nissan ENV200 and Renault Twizy
* More information [here](https://github.com/project-aslan/Aslan/tree/master/src/vehicle_configuration)

### vehicle_interface
#### sd_vehicle_interface:
Originally developed by StreetDrone     
**Features:** 
* A the bridge between ROS and the StreetDrone embedded system (XCU) CAN input. Control loop feedback mechanisms are also implemented in the interface for vehicle speed and steer control. 
* Implemented PID/Feedforward linear velocity feedback loop within the control library  
* Continuous software to Drive-by-Wire (DBW) system handshake verification
* High level safety monitoring with fault response from the supervisor package output
* StreetDrone XCU interface for the CAN Bus of both StreetDrone Twizy and StreetDrone ENV200
* Tunable yaw to steering percentage conversion map 
* GPS/IMU configuration   
* Integration of GPS and IMU vehicle pose information  
* Simulation mode enabling   
* Specific simulation configuration 
* More information [here](https://github.com/project-aslan/Aslan/tree/master/src/vehicle_interface)

### vehicle_simulation/gazebo
#### sd_twizy_model:
Originally developed by StreetDrone    
**Features:**
* Gazebo simulation packages for the streetdrone twizy vehicle
* Aslan software stack output simulation
* sd_vehicle_interface control 
* Joystick control: The robot supports the generic Linux joystick controllers.
* Keyboard control
* Sensors support:   
LiDAR: VLP - 16 Velodyne  
Cameras: 8 x Blackfly S 2.3MP  
* Osney point cloud Gazebo world. This world was developed using a recorded lidar point cloud and it is being integrated in Gazebo simulation.
* Default rviz configuration file
* More information [here](https://github.com/project-aslan/Aslan/tree/master/src/vehicle_simulation/gazebo/sd_twizy_model)

### aslan_msgs: 
ProjectAslan specific development. Consists of msgs included in autoware_config_msgs and autoware_msgs as well as aslan specific msgs, in order to accommodate the message passing and parsing requirements of the Aslan source code and to fit the Aslan architecture.    
**Changes made:** 
* Added radar object detection configuration messages  
* SD vehicle control messages  

### autoware.ai:
Submodule pointing to a fork of autoware.ai open source software for self-driving vehicles.  

#### localization/lidar_localizer:
For the vehicle to accurately estimate its position within an environment, autoware.ai is suggesting a point cloud map-based localisation method which StreetDrone has evaluated in terms of accuracy, resilience to error and noise, coverage and cost, before integrating in Aslan. This localization approach is using the point cloud matching method, Normal Distribution Transform or NDT in which a LiDAR scan from the vehicle is being compared to the LiDAR scan of the map, in order to calculate its position in that map.   
**Added Features for Aslan:**   
* The node save_pcd is a Project-Aslan specific development. This package is responsible for saving a point cloud map at a specific path. It's being automatically run inside ndt_mapping.
* Launch pose_relay and vel_relay from [topic_tools/relay](https://github.com/ros/ros_comm/tree/melodic-devel/tools) alongside ndt_matching [(http://wiki.ros.org/topic_tools/relay)](http://wiki.ros.org/topic_tools/relay)
* ndt_matching: Publish predict_pose_error (the difference between ndt_pose and predict_pose)   
* More information [here](https://github.com/project-aslan/autoware.ai/tree/aslan-dev/localization/packages/lidar_localizer)

#### mission/lane_planner: 
This package is responsible for extracting the route for the vehicle to follow. It is using pre-defined waypoints in the form of lane arrays and then it is drawing the route by finding the closest waypoint from the vehicle's current position in the map.   
**Added Features for Aslan:**  
* The node traffic_waypoints is Project Aslan specific development. This node is responsible for publishing the traffic waypoints array from the lane waypoints array.   
* More information [here](https://github.com/project-aslan/autoware.ai/tree/aslan-dev/planning/mission/packages/lane_planner)

#### motion/astar_planner:
This package is based on the A* search algorithm for path planning. The A* algorithm is using a cost function for finding the optimal way of reaching the next target waypoint. It's considered to be the best planning algorithm for finding the shortest path to target.  
**Added Features for Aslan:**  
* Radar obstacle detection
* Emergency reaction and emergency stop from radar obstacle detection
* Deceleration and search distance calibration
* Added obstacle removed reaction
* Parameters configuration: Added radar detection specific launch parameters, added current_velocity input topic definition, added obstacle detection additional launch parameters, specified default parameters setting after on-vehicle testing, removed crosswalk detection.   
* Configurable speed input topic
* More information [here](https://github.com/project-aslan/autoware.ai/tree/aslan-dev/planning/motion/packages/astar_planner)

#### motion/waypoint_follower: 
This package is responsible for calculating the final velocity command for the vehicle, by using the output from the path planning module and fitting a curve between the car and the next target waypoint, based on the lookahead distance specified.  
**Added Features for Aslan:**  
* Added emergency stop reaction
* Vehicle speed conditioning to fit the requirements of Project Aslan: 9mps speed cap (at pure_pursuit).  
* Obstacle removed callback reaction, added zero_twist counter after on-vehicle testing to capture unexpected message dropping to 0 (at twist filter). 
* More information [here](https://github.com/project-aslan/autoware.ai/tree/aslan-dev/planning/motion/packages/waypoint_follower)

#### motion/waypoint_maker: 
This package is responsible for extracting and then loading the waypoints for the vehicle to follow. Each waypoint is associated with (x,y,z) coordinates relative to a common reference point and a velocity value.  
* More information [here](https://github.com/project-aslan/autoware.ai/tree/aslan-dev/planning/motion/packages/waypoint_maker)

#### filters
* voxel_grid filter: This package is responsible for downsampling the input point cloud data produced by the lidar using the voxel grid filtering technique. More information [here](https://github.com/project-aslan/autoware.ai/tree/aslan-dev/filters/voxel_grid_filter)
* ray_ground_filter: This package is responsible for filtering the output point cloud from a sensor (either lidar or radar). It is responsible for performing ground removal, by masking the point cloud received. More information [here](https://github.com/project-aslan/autoware.ai/tree/aslan-dev/filters/ray_ground_filter)


#### Overall changes made:
* All path planning and controls parameters are configured to default values after on-vehicle testing
* NDT doesn't include GPU support
* Aslan doesn't include vector maps support. Project Aslan is tested and proven stable with point cloud maps only.   
* Aslan doesn't support crosswalk detection, traffic lights, mode and gear command


### sensing/drivers:
#### camera:
Includes the following submodules: 
* [camera_control_msgs:](https://github.com/magazino/camera_control_msgs) Originally developed by Magazino GmbH using the pylon Software Camera Suite by Basler AG.   
* [pointgrey_camera_driver:](https://github.com/ros-drivers/pointgrey_camera_driver) Originally provided by Carnegie Mellon University  
* [pylon_camera:](https://github.com/magazino/pylon_camera) Originally developed by Magazino GmbH using the pylon Software Camera Suite by Basler AG 
* [spinnaker_sdk_camera_driver:](https://github.com/neufieldrobotics/spinnaker_sdk_camera_driver) Originally provided by NEU Field Robotics Lab 
* [zed-ros-wrapper:](https://github.com/stereolabs/zed-ros-wrapper) Originally provided by StereoLabs     
  
#### lidar:
* [ouster:](https://github.com/project-aslan/ouster_example) Submodule pointing to a fork of ouster-lidar/ouster_example.    
**Changes made:** Remapped the lidar topics topic to /points_raw  

* [velodyne:](https://github.com/project-aslan/velodyne) Submodule pointing to a fork of ros-drivers/velodyne
**Changes made:** Project Aslan only includes the sensors that have been tested and proven successful with the stack. These are the lidars: Velodyne-VLP16 and VLP-32.  

#### radar: 
[umrr_driver:](https://www.smartmicro.com/downloads) Originally provided by Smartmicro


