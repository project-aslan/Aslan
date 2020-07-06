
```
aslan_docker: Aslan docker container linking to projaslan/aslan in Dockerhub 
(https://hub.docker.com/r/projaslan/aslan)

src
└───aslan_tools
│   └───aslan_build_flags: Default build flags
│   │ 
│   └───aslan_gui: A Graphical User Interface (GUI) aiming to facilitate the user when using Aslan.
│   │	[Features:] Nodes launching, parameters configuration, ROS visualization tools (rqt, rviz), 
│   │	data visualization, rosbag manipulation tools, Gazebo simulation launching and built-in documentation.
│   │	Tools used: wxGlade and wxPython. The package runtime_manager from Autoware.ai has been a great inspiration 
│   │	when developing this package.
│   │
│   └───rviz: Default rviz configuration file for data visualization in Aslan
│
└───msgs/aslan_msgs: Project-Aslan specific development. 
|   Consists of msgs included in autoware_config_msgs and autoware_msgs as well as project-aslan  
|   specific msgs, in order to accommodate the message passing and parsing requirements of the 
|   Project-Aslan source code and to fit the Project-Aslan architecture. Changes made:
│  	  * Added radar messages
│  	  * SD vehicle control messages
│
└───supervisor/ check_messages_alive: Developed by StreetDrone and donated to Project Aslan
|	[Features:] High level supervisor node, monitoring the health of the system and publishing 
|	diagnostic messages for each package. This package is tracking the status of some topics and 
|	publishing an array of the status of each topic in topic /sd_diagnostics using a diagnostics 
|	array message. 
|
└───vehicle_configuration
|    └───models/vehicle_model: Developed by StreetDrone and donated to Project Aslan
|    |	[Features:] Vehicle models visualization in rviz using 3D CAD models. Supports:
|    |		* Nissan ENV200
|    |		* Renault Twizy	
|    |   	
|    └───vehicle_info/yaml_files: Developed by StreetDrone and donated to Project Aslan
|    	[Features:] Vehicles description parameters
|
└───vehicle_interface/sd_vehicle_interface: Originally developed by StreetDrone 
|	[Features:] A the bridge between ROS and the StreetDrone embedded system (XCU) CAN input.
|	Control loop feedback mechanisms are also implemented in the interface for vehicle speed 
|	and steer control.
|	(https://github.com/streetdrone-home/SD-VehicleInterface). Changes made:
│  		* Implemented PID/Feedforward linear velocity feedback loop within the control library
│  		* Configurable yaw map
│  		* GPS/IMU configuration
│  		* Configurable vehicle speed input
│  		* Throttle and steer commands
│  		* Simulation mode enabling
│  		* Configured for simulation
|
└───vehicle_simulation/gazebo/sd_twizy_model
|     └───sd_robot: Originally developed by StreetDrone
|     |	[Features:] * Gazebo simulation packages for the sd twizy
|     |		    * Aslan stack output simulation
|     |		    * Joystick control
|     |		    * Keyboard control
|     |		    * Multiple sensors support
|     |	(https://github.com/streetdrone-home/SD-TwizyModel). Changes made:
|     |    * Added Osney point cloud Gazebo world
|     |    * Added silicon_osney world. This world was developed using a recorded lidar point cloud
|     |       and it is being integrated in Gazebo simulation
|     |
|     └───sd_control: Originally included at StreetDrone
|     |   (https://github.com/streetdrone-home/SD-TwizyModel). Changes made:
|     |   * Control from Aslan output 
|     |   * Default rviz configuration file
|     |   * Keyboard control
|     |
|     └───sd_description:Originally developed by StreetDrone
|          (https://github.com/streetdrone-home/SD-TwizyModel). Contents have remained unchanged
│
└───autoware.ai: Fork from autoware-ai open source software for self-driving vehicles
|     └───localization
|     │  └───lib: 
|     |  |    ndt_cpu: Developed by Point Cloud Library (PCL) and Autoware 1.10.0. (https://github.com/PointCloudLibrary/pcl). 
|     |  |    Changes made:
|     |  |          * fixed copyrights to include Point Cloud Library (PCL)
|     |  |
|     |  |    pcl_omp_registration: Originally developed by Point Cloud Library (PCL) (https://github.com/PointCloudLibrary/pcl). 
|     |  |     This package has remained unchanged.
|     │  | 
|     │  └───packages/lidar_localizer: 
|     │       The node save_pcd is a Project-Aslan specific developement.
|     │       The nodes ndt_mapping, ndt_matching, ndt_matching_monitor were originally included in Autoware 1.10.0. 
|     │       Changes made:
|     │  		* Launch pose_relay and vel_relay from topic_tools/relay (https://github.com/ros/ros_comm/tree/kinetic-devel/tools) 
|     │  		  alongside ndt_matching (http://wiki.ros.org/topic_tools/relay) 
|     │  		* ndt_mapping: Doesn't support GPU for NDT
|     │  		* ndt_matching:  Doesn't support GPU for NDT. Publishing predict_pose_error 
|     |			(the difference between ndt_pose and predict_pose)
|     │ 
|     └───mapping
|     |   └───map_tf_generator: This package has remained unchanged.
|     |   |     
|     |   └───pcd_loader: Originally included in Autoware.ai 1.10.0 under package /map_file. Changes made:
|     |		* Doesn't support vector maps. Project Aslan is tested and proven stable with point cloud maps only.
|     | 
|     └───planning
|     |   └───mission/packages/lane_planner
|     │   |	The node traffic_waypoints is Project-Aslan specific development.
|     |   |	[Feature:] This node is publishing the traffic waypoints array.
|     │   |	The node lane_select is included in Autoware.ai 1.10.0.
|     |   |     
|     |   └───motion
|     |       └───astar_planner: Included in Autoware.ai 1.10.0. Changes made:
|     │       │   Changes made:
|     │       │     * velocity_set parameters: Added radar detection specific launch parameters, added current_velocity 
|     │       │     input topic definition, added obstacle detection additional launch parameters, specified
|     │       │     default parameters setting after on-vehicle testing, removed crosswalk detection.
|     │       │     * velocity_set: 
|     |       |         * [Feature] Configurable speed input topic
|     │       │         * [Feature] Radar obstacle detection
|     │       │         * [Feature] Emergency reaction to obstacle detection from radar 
|     │       │         * Calibration for deceleration and search distance after on-vehicle testing 
|     │       │         * [Feature] Emergency stop, obstacle detection, obstacle removed
|     │       │         * Doesn't support vector maps and crosswalk detection
|     │       │     
|     |       └───waypoint_follower: Included in Autoware.ai 1.10.0. Changes made:
|     │       │     * [Feature] pure_pursuit - Vehicle speed conditioning to fit the requirements of 
|     │       │     Project-Aslan: 9mps speed cap.
|     │       │     * [Feature] twist_filter - Obstacle removed callback reaction, added zero_twist counter after 
|     │       │     on-vehicle testing to capture unexpected message dropping to 0
|     │       │     * twist_gate: Doesn't support mode and gear command
|     │       │     * Fixed warning messages when building
|     │       │     
|     |       └───waypoint_maker: Originally included in Autoware.ai 1.10.0. Changes made:
|     │             * waypoint_marker_publisher: Doesn't support traffic lights
|     │             * Fixed warning messages when building
|     │             * Parameters calibration from on-vehicle testing
|     |
|     └───filters
|          └───ray_ground_filter: Originally included in Autoware.ai 1.10.0 under points_preprosessor. 
|          |    Changes made:
|          |       * Parameters calibration from on-vehicle testing
|          |  
|          └───voxel_grid_filter: Originally included in Autoware.ai 1.10.0 under points_downsampler. 
|       	Changes made:
|       	   * Parameters calibration from on-vehicle testing
|
└───sensing
|   └───drivers
|    |  └───camera: Includes the following submodules
|    |  |   └───camera_control_msgs: Originally developed by Magazino GmbH using the pylon 
|    |  |   |   	Software Camera Suite by Basler AG (https://github.com/magazino/camera_control_msgs). 
|    |  |   |
|    |  |   └───pointgrey_camera_driver: Originally provided by Carnegie Mellon University 
|    |  |   |   	(https://github.com/ros-drivers/pointgrey_camera_driver)
|    |  |   |
|    |  |   └───pylon_camera: Originally developed by Magazino GmbH using the pylon 
|    |  |   |   	Software Camera Suite by Baselr AG (https://github.com/magazino/pylon_camera)
|    |  |   |
|    |  |   └───spinnaker_sdk_camera_driver: Originally provided by NEU Field Robotics Lab 
|    |  |   |   	(https://github.com/neufieldrobotics/spinnaker_sdk_camera_driver)
|    |  |   |
|    |  |   └───zed-ros-wrapper: Originally provided by StereoLabs 
|    |  |       	(https://github.com/stereolabs/zed-ros-wrapper)
|    |  |
|    |  └───lidar
|    |  |   └───ouster: Originally provided by ouster-lidar 
|    |  |   |		(https://github.com/ouster-lidar/ouster_example). Changes made:
|    |  |   |      		* Remapped the lidar topics topic to /points_raw
|    |  |   |  
|    |  |   └───velodyne: Originally provided by Austin Robot Technology 
|    |  |     		(https://github.com/ros-drivers/velodyne). Changes made:
|    |  |           * Project-Aslan only includes the lidars that have been tested and proven successful with the stack. 
|    |  |        	  These are the sensors: Velodyne-VLP16 and VLP-32.
|    |  |   
|    |  └───radar
|    |   	└───umrr_driver: Originally provided by Smartmicro (https://www.smartmicro.com/downloads)
_
```

