Folder structure

```
aslan_docker: Project-Aslan specific development linking to projaslan/aslan in Dockerhub 
(https://hub.docker.com/r/projaslan/aslan)

src
└───aslan_tools
│   └───aslan_build_flags: Project-Aslan specific development
│   │ 
│   └───aslan_gui: Project-Aslan specific development. Tools used: wxGlade and wxPython. 
│   │   The package runtime_manager from Autoware.ai 1.10.0 has been a great inspiration when developing this package.
│   │
│   └───rviz: Default rviz configuration file for Project-Aslan specific development
│
└───localization
│  └───lib: 
|  |    ndt_cpu: Developed by Point Cloud Library (PCL) and Autoware 1.10.0. (https://github.com/PointCloudLibrary/pcl). 
|  |    Changes made:
|  |          * fixed copyrights to include Point Cloud Library (PCL)
|  |
|  |    pcl_omp_registration: Originally developed by Point Cloud Library (PCL) (https://github.com/PointCloudLibrary/pcl). 
|  |     This package was originally included in Autoware 1.10.0 and has remained unchanged.
│  | 
│  └───packages/lidar_localizer: 
│       The node save_pcd is a Project-Aslan specific developement.
│       The nodes ndt_mapping, ndt_matching, ndt_matching_monitor were originally included in Autoware 1.10.0. 
│       Changes made:
│  		* Removed queue_counter package
│  		* Set default use of gnss to False
│  		* The nodes pose_relay and vel_relay from topic_tools/relay (https://github.com/ros/ros_comm/tree/kinetic-devel/tools) 
│  		  are automatically launched from the ndt_matching launch file (http://wiki.ros.org/topic_tools/relay) 
│  		  converting the topics: /ndt_pose to /current_pose and /estimate_twist to /current_velocity
│  		* ndt_mapping.cpp: Removed all references to CUDA implementation as this version of Aslan  
│  		  doesn't support GPU for NDT. Changed to use the aslan_msgs package.
│  		* ndt_matching.cpp: Removed all references to CUDA implementation as Aslan doesn't support 
│  		  GPU for NDT. Changed to use the aslan_msgs package
│  		* ndt_matching.cpp: Added the topic /rmse of type: std_msgs/Float, to publish the predict_pose_error (
│  		  the difference between ndt_pose and predict_pose)
│  		* ndt_matching_monitor: Changed to use the aslan_msgs package
│ 
└───mapping
|   └───map_tf_generator: Originally included in Autoware.ai 1.10.0. This package has remained unchanged.
|   |     
|   └───pcd_loader: Originally included in Autoware.ai 1.10.0 under package /map_file. Changes made:
│  		* Points_map_filter, vector_map loader have been removed as this version of Aslan doesn't support vector maps
│  		* get_file.cpp: Usage of pkg aslan_msgs instead of autoware_msgs to fit the ASLAN architecture. 
│  		* The nodes points_map_filter and vector_map_loader have been removed.
|        
└───msgs/aslan_msgs: Project-Aslan specific development. 
|   Consists of msgs included in autoware_config_msgs and autoware_msgs as well as project-aslan  
|   specific msgs, in order to accommodate the message passing and parsing requirements of the 
|   Project-Aslan source code and to fit the Project-Aslan architecture. Changes made:
│  	  * ConfigVelocitySet.msg: Added radar threshold points
│  	  * SDControl.msg: Project-Aslan specific message for the SD-VehicleInterface in Gazebo sim mode 
│  	  * Changed to Project-Aslan folder structure
| 
└───planning
|   └───mission/lane_planner
│   |	The node traffic_waypoints is Project-Aslan specific development.
│   |	The node lane_select is included in Autoware.ai 1.10.0. Changes made:
│   |     * lane_select.launch automatically launches the traffic_waypoints node
│   |     * Changed to aslan_msgs package
|   |     
|   └───motion
|       └───astar_planner: Originally included in Autoware.ai 1.10.0. Changes made:
│       │     * obstacle_avoid: Removed obstacle_sim automatic launching
│       │     * Added obstacle_sim.launch file
│       │     * velocity_set.launch: Added radar detection specific launch parameters, added current_velocity 
│       │     input topic definition, added obstacle detection additional launch parameters, specified
│       │     default parameters setting after on-vehicle testing, removed unused crosswalk detection.
│       │     * obstacle_avoid: Changed to use the aslan_msgs package
│       │     * libvelocity_set: Removed crosswalk detection and unused code
│       │     * velocity_set.cpp: 
|       |         * Changed decelerate for obstacle calibration after on-vehicle testing
|       |         * Added configurable speed input topic
│       │         * Added radar obstacle detection code
│       │         * Added emergency reaction to obstacle detection from radar 
│       │         * Defined default parameters for deceleration and search distance after on-vehicle testing 
│       │         * Added flags for emergency stop, obstacle detection, obstacle removed
│       │         * Added parameter getAccelerateMax and removed getDetectionObstacle
│       │         * Removed crosswalk detection, added comments on the code and cleaned-up the code
│       │         * Removed vector maps
│       │     * velocity_set_info.cpp: Added radar detection callback functions, added default configuration 
│       │     and launch parameters
│       │     * velocity_set_info.h: Added radar detection callback functions, added default configuration 
│       │     and launch parameters
│       │     * velocity_set_path.cpp: Changed default parameters after on-vehicle testing
│       │     * velocity_set_path.h: Changed to use the aslan_msgs package
│       │     
|       └───waypoint_follower: Originally included in Autoware.ai 1.10.0. Changes made:
│       │     * pure_pursuit_core.cpp: Added vehicle speed conditioning to fit the requirements of 
│       │     Project-Aslan and increased subscriber's buffer. Added a 9mps speed cap.
│       │     * twist_filter.cpp: Added obstacle removed callback, added zero_twist counter after 
│       │     on-vehicle testing to capture unexpected message dropping to 0
│       │     * twist_gate.cpp: Removed mode and gear command callback, as it's unused code
│       │     * Changed to use the aslan_msgs package
│       │     * Fixed warning messages when building
│       │     * Changed default parameters after on vehicle testing
│       │     * Removed wf_simulator
│       │     
|       └───waypoint_maker: Originally included in Autoware.ai 1.10.0. Changes made:
│             * waypoint_marker_publisher: Removed traffic light colour callback functions
│             * Changed to aslan_msgs package, changed default parameters and removed catkin build warnings
│             * Removed waypoint_clicker, waypoint_velocity_vizualizer nodes
|       
└───sensing
|   └───drivers
|    |  └───camera
|    |  |   └───camera_control_msgs: Originally developed by Magazino GmbH using the pylon 
|    |  |   |   	Software Camera Suite by Basler AG (https://github.com/magazino/camera_control_msgs). 
|    |  |   |   	The contents of the packages included under camera_control_msgs have remained unchanged
|    |  |   |
|    |  |   └───pointgrey_camera_driver: Originally provided by Carnegie Mellon University 
|    |  |   |   	(https://github.com/ros-drivers/pointgrey_camera_driver)
|    |  |   |   	The contents of the packages included under at pointgrey_camera_driver have remained unchanged
|    |  |   |
|    |  |   └───pylon_camera: Originally developed by Magazino GmbH using the pylon 
|    |  |   |   	Software Camera Suite by Baselr AG (https://github.com/magazino/pylon_camera)
|    |  |   |   	The contents of the packages included under pylon_camera have remained unchanged
|    |  |   |
|    |  |   └───spinnaker_sdk_camera_driver: Originally provided by NEU Field Robotics Lab 
|    |  |   |   	(https://github.com/neufieldrobotics/spinnaker_sdk_camera_driver)
|    |  |   |   	The contents of the packages included under spinnaker_sdk_camera_driver have remained unchanged
|    |  |   |
|    |  |   └───zed-ros-wrapper: Originally provided by StereoLabs 
|    |  |       	(https://github.com/stereolabs/zed-ros-wrapper)
|    |  |       	The contents of the packages included under zed-ros-wrapper have remained unchanged
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
|    |       	The contents of the packages included under umrr_driver have remained unchanged
|    |
|    └───filters
|       	└───ray_ground_filter: Originally included in Autoware.ai 1.10.0 under points_preprosessor. 
|       	|    Changes made:
|       	|       * Updated default parameters after on-vehicle testing and changed to aslan_msgs pkg
|       	|       * Removed cloud_transformer, compare_map_filter, points_concat_filter, 
|       	|       ring_ground_filter, space_filter
|       	|  
|       	└───voxel_grid_filter: Originally included in Autoware.ai 1.10.0 under points_downsampler. 
|       	     Changes made:
|       	        * Updated default parameters after on-vehicle testing, changed the published topic to voxel 
|       	          grid filter info and changed to aslan_msgs package
|       	        * Removed distance filter, random filter, ring filter
|
└───supervisor/ check_messages_alive: Originally developed by StreetDrone and donated to Project Aslan
|
└───vehicle_configuration
|    └───models/vehicle_model: Originally developed by StreetDrone and donated to Project Aslan
|    |   	
|    └───vehicle_info/yaml_files: Originally developed by StreetDrone and donated to Project Aslan
|
└───vehicle_interface/sd_vehicle_interface: Originally developed by StreetDrone 
|	(https://github.com/streetdrone-home/SD-VehicleInterface). Changes made:
│  		* Implemented PID/Feedforward linear velocity feedback loop within the control library
│  		* Configurable yaw map
│  		* Added GPS/IMU configuration, input velocity configuration,
│  		* Removed msgs package
│  		* Changed to use aslan_msgs
│  		* Added various vehicle speed input topics
│  		* Added throttle and steer command to SDControl message
│  		* Added simulation mode enabling
│  		* Configured for simulation
│  		* Added SDControl message passing and parsing
|
└───vehicle_simulation/gazebo/sd_twizy_model
|     └───sd_robot: Originally developed by StreetDrone
|	(https://github.com/streetdrone-home/SD-TwizyModel). Changes made:
|     |    * Added Osney point cloud Gazebo world
|     |    * Added sd_twizy_osney launch file and sd_twizy_world generic launch file.
|     |    * Added silicon_osney world. This world was developed using a recorded lidar point cloud
|     |       and it is being integrated in Gazebo simulation
|     |
|     └───sd_control: Originally included at StreetDrone
|     |   (https://github.com/streetdrone-home/SD-TwizyModel). Changes made:
|     |   * Changed the default rviz configuration file to fit the requirements of Project-Aslan
|     |   * Changed to use aslan_msgs
|     |   * Added keyboard control script
|     |
|     └───sd_description:Originally developed by StreetDrone
|          (https://github.com/streetdrone-home/SD-TwizyModel). Contents have remained unchanged

_
```

