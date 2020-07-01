umrr_driver 1.1.0
=======================

Overview/Purpose
-----------------------

This package provides a ROS node to convert the target list of a radar sensor into a PointCloud2 message and
nodes to further process the point clouds. The main node `umrr_can_publisher`
configures the radar sensor and reads the incoming CAN messages.
Also provided is an example for 3D Visualization using Rviz.

### Hardware prerequisites
- smartmicro automotive radar sensor (T132) with software > 2.2.0 or T152 with software > 0.6 
- Lawicel USBCAN adapter or any other adapter compatible with linux providing an can socket

### Configuration
This driver is designed for Ubuntu 16.04 LTS running ROS Kinectic.  
Also tested on Ubuntu 18.04 LTS running ROS Melodic.
CAN Protocol format: json configuration file

### Test Environment
Tested with Lawicel USBCAN Adapter using umrr8f and umr11.

### Build Tool
The package uses catkin build tools.


Installation
-----------------------
The node operates using a can socket. If you are not using the LAWICEL adapter make sure
the socket is reachable for example using the command
candump slcan0

The Lawicel adapter can be installed using the following steps:

1. add the kernel module slcan to your etc/modules file so it is loaded during booutup
2. check if the module is running by typing `lsmod|grep slcan`
output should look similar to: `slcan   16834 0`
3. install can-utils by typing  
`sudo apt install can-utils`
4. Map the usb on a can device by running
sudo slcand -o -c -f -s6 /dev/ttyUSB0 slcan0
5. Setup the network device making the socket available for the userspace  
`sudo ifconfig slcan0 up`
6. Test if the setup is done and working correctly by typing  
`candump slcan0`  
You should see the raw can messages

This node requires python 3 while ROS only supports python 2.7 therefore we need to
set up an virtual environment.

Prerequisites:
install the following packages using apt-get:
- python3-dev
- virtualenvwrapper
- virtualenv

Close your shell afterwards to update your .bash_rc

1. Create an virtual enviroment using:  
`mkvirtualenv -p /usr/bin/python3 <your_enviroment_name>`  
This creates an enviroment and also activates it. Notice the brackets in your command line

2. install the following packages in your virtual enviroment
- rospkg
- catkin_pkg
- crc_mod  

    using the command:  
    `pip install rospkg catkin_pkg crcmod`

Leave your virtual enviroment.

For convenient use e.g. nest the launch file in your existing project, you will have to edit
the umrr_can_publisher script.
Open the python script located in umrr_drivers/src and edit the shebang line.

Enter the path to the enviroment you just setup. To do this:
1. Open a new terminal
2. type `workon <name_of_your_enviroment>`
3. type `echo $VIRTUAL_ENV` copy the resulting path and leave the console
4. enter the obtained path to your shebang path for example:
`#!/home/user/.virtualenvs/ros_driver/bin/python`


Usage
-----------------------

The `umrr_can_publisher` node needs Python3, while ROS officially only supports Python2.
So make sure you are starting this node with Python3, for example by using a virtual environment.
For convenience edit the shebang line in the `umrr_can_publisher` script to suit your setup.

Make sure the can socket is available to linux and a sensor is connected. If you are
not using socket
slcan0, edit the nodes configuration file.

1. Open a console and type:  
`roslaunch umrr_driver automotive_radar.launch`

3. Incoming data should now be visible to check type  
`rostopic echo /radar_data`

4. Visualization can be started using  
`roslaunch umrr_driver visualization.launch`

Config files
----------------------

cfg/
- `example_config.yaml` example setup file for the umrr_can_publisher.
- `MSE_config.yaml` setup file for using a UMRR11 MSE type sensor.
- `pc2filter.cfg` sets the configurable parameters for the rqt server of pc2filter node


Launch Files
----------------------

- launch file `visualization.launch` is an example for a basic visualization using Rviz
- launch file `automotive_radar.launch` starts the umrr_can_publisher node with the given
config file.
    Arguments:  
    - `config_file` sets the path to the parameter config.   
    Default: `$(find umrr_driver)/cfg/example_config.yaml`
    - `can_spec` sets the path to the can spec.json  
    Default: `$(find umrr_driver)/can_spec/automotive_spec.json`
    - `node_namespace` sets the nodes namespace. Useful when running more then one
    instance of the umrr_can_publisher node and its pipeline.  
    Default: `radar`
- launch file `automotive_mse.launch` is an example for using the umrr_driver with the automotive_mse can spec.    


Nodes
----------------------

### umrr_can_publisher

This node reads the incoming CAN messages, writes the data to a PointCloud2 message using the `can_spec` and publishes it.

**Important:** This node need Python3, while ROS officially only supports Python2.
So make sure you are starting this node with Python3, for example by using a virtual environment.

#### Published Topics

* **`/radar_data`** (sensor_msgs/PointCloud2)

  A point cloud with the radar targets of the current cycle. Depending on the sensor configuration, the point cloud
  will contain different fields, which are defined in the corresponding `can_spec`.

#### Services  
- `sensor_status` (srv/sens_status)  
Send a status request to the sensor. Returns the corresponding value. Usage:
`rosservice call /node_namespace/sensor_status section parameter`
- `sensor_parameter` (srv/sens_param)
Send a parameter write to the sensor. Sets the corresponding value. Usage:  
`rosservice call /node_namespace/sensor_parameter section parameter value`
- `sensor_parameter_request` (srv/sens_param_req)
Send a parameter request to the sensor. Returns the corresponding value. Usage:  
`rosservice call /node_namespace/sensor_parameter_request section parameter`

#### Parameters

* **`~can_spec`** (string)

  Path to the `can_spec` to use.

* **`~frame_id`** (string, default:"radar")

  Frame ID of the sensor.

* **`~can_socket`** (string, default:"slcan0")

  Can socket the node should use.

* **`~legacy_mode`** (int, default: 0, min: 0, max: 1)

  Choose to start node in legacy mode. If set, communication with sensor except for
  receiving a target list is switched off. Also the services `sensor_status` and
  `sensor_parameter` are not offered.

* **`~antenna_mode`** (dict, default:"")

  Provide a dict to configure the sensors used antenna on startup. Node expects following order:
  `section, parameter, value`

* **`~center_frequency`** (dict, default:"")

  Provide a dict to configure the sensors used center frequency on startup. Node expects following order:
  `section, parameter, value`

###  Pc2_filter
A node to filter the targets coming from the sensor. To change values during runtime this
node offers a dynamic reconfigure server.

#### Subscribed Topics

* **`/radar_data`** (sensor_msgs/PointCloud2)

  The input cloud must contain at least the following fields:
  * `Range` (Unit: [m])
  * `Azimuth` (Unit: [°])
  * `Elevation` (Unit: [°])
  * `Speed_Radial` (Unit: [m/s])


#### Published Topics

* **`/filtered_data`** (sensor_msgs/PointCloud2)

    The output cloud contains the same fields as `/radar_data`


### Spherical_Coord_2_Cartesian_Coord

This node converts a point cloud with targets in spherical coordinates to a point cloud with cartesian coordinates,
so that it is possible to view the point cloud in Rviz.

#### Subscribed Topics

* **`filtered_data`** (sensor_msgs/PointCloud2)

  The input cloud must contain the following fields:
  * `Range` (Unit: [m])
  * `Azimuth` (Unit: [°])
  * `Elevation` (Unit: [°])
  * `Speed_Radial` (Unit: [m/s])


#### Published Topics

* **`/target_list_cartesian`** (sensor_msgs/PointCloud2)

  The output cloud contains the following fields:
  * `x` (Unit: [m])
  * `y` (Unit: [m])
  * `z` (Unit: [m])
  * `Speed_Radial` (Unit: [m/s])
  * `RCS` (Unit: [dB/m])
  * `SNR` (Unit: [dB])  

### mse_converter  

Node is used to reorder the pointcloud fields which are read in from CAN in order to display it in RVIZ.

#### Subscribed Topics

* **`/radar_data`** (sensor_msgs/PointCloud2)

#### Published Topics

* **`/object_list`** (sensor_msgs/PointCloud2)


Notes
----------------------
If you have a sensor with software below the required version or a UMRR-8F, you can use
the legacy mode of the driver.

Bugs & Feature Requests
----------------------
Please report bugs and feature requests to: `support@smartmicro.de`
